/**
 * linked_list.c — Malloc-free outbox / event-log linked list.
 *
 * Uses an embedded static node pool (list->pool[LL_POOL_SIZE]) managed by an
 * O(1) free-list (list->free_head). Zero heap allocation (malloc/free).
 * Persistence is routed through hal_storage (LittleFS on ESP32, file on sim).
 */
#include "linked_list.h"
#include "hal.h"
#include <stdio.h>
#include <string.h>

void ll_init(LinkedList *list)
{
    size_t i;
    if (!list)
        return;
    memset(list, 0, sizeof(*list));
    /* Thread all static pool slots into the O(1) free-list */
    for (i = 0; i < LL_POOL_SIZE - 1; i++) {
        list->pool[i].next = &list->pool[i + 1];
    }
    list->pool[LL_POOL_SIZE - 1].next = NULL;
    list->free_head = &list->pool[0];
}

void ll_clear(LinkedList *list)
{
    /* O(1) reset: re-thread static pool into free_head. */
    if (!list)
        return;
    ll_init(list);
}

int ll_is_empty(const LinkedList *list)
{
    return !list || list->head == NULL;
}

size_t ll_count(const LinkedList *list)
{
    return list ? list->count : 0;
}

size_t ll_pool_free_count(const LinkedList *list)
{
    return list ? (LL_POOL_SIZE - list->count) : 0;
}

/* Allocate one node from the static free-list in O(1). */
static LinkedListNode *alloc_node(LinkedList *list, const Alert *alert)
{
    LinkedListNode *n;
    if (!list || !list->free_head)
        return NULL;
    n = list->free_head;
    list->free_head = n->next;
    n->data = *alert;
    n->next = NULL;
    return n;
}

/* Return one node back to the static free-list in O(1). */
static void release_node(LinkedList *list, LinkedListNode *n)
{
    if (!list || !n)
        return;
    n->next = list->free_head;
    list->free_head = n;
}

int ll_append(LinkedList *list, const Alert *alert)
{
    LinkedListNode *n;
    if (!list || !alert)
        return -1;
    n = alloc_node(list, alert);
    if (!n)
        return -1;
    if (!list->head) {
        list->head = list->tail = n;
    } else {
        list->tail->next = n;
        list->tail = n;
    }
    list->count++;
    return 0;
}

int ll_insert_by_severity(LinkedList *list, const Alert *alert)
{
    /*
     * O(n): walk until next node has lower severity, then splice in.
     * Result: CRITICAL ... HIGH ... MEDIUM ... LOW (FIFO within same severity).
     */
    LinkedListNode *n, *prev, *cur;
    if (!list || !alert)
        return -1;
    n = alloc_node(list, alert);
    if (!n)
        return -1;

    if (!list->head) {
        list->head = list->tail = n;
        list->count++;
        return 0;
    }

    /* Insert before first node with strictly lower severity. */
    prev = NULL;
    cur = list->head;
    while (cur && (int)cur->data.severity >= (int)alert->severity) {
        prev = cur;
        cur = cur->next;
    }

    if (!prev) {
        /* New head (highest severity so far). */
        n->next = list->head;
        list->head = n;
    } else {
        n->next = prev->next;
        prev->next = n;
        if (!n->next)
            list->tail = n;
    }
    list->count++;
    return 0;
}

int ll_remove_by_id(LinkedList *list, int id)
{
    LinkedListNode *prev = NULL, *cur;
    if (!list)
        return -1;
    cur = list->head;
    while (cur) {
        if (cur->data.id == id) {
            if (!prev)
                list->head = cur->next;
            else
                prev->next = cur->next;
            if (list->tail == cur)
                list->tail = prev;
            release_node(list, cur);
            list->count--;
            if (list->count == 0)
                list->head = list->tail = NULL;
            return 0;
        }
        prev = cur;
        cur = cur->next;
    }
    return -1;
}

Alert *ll_find(LinkedList *list, int id)
{
    LinkedListNode *cur;
    if (!list)
        return NULL;
    for (cur = list->head; cur; cur = cur->next) {
        if (cur->data.id == id)
            return &cur->data;
    }
    return NULL;
}

Alert *ll_peek_head(LinkedList *list)
{
    if (!list || !list->head)
        return NULL;
    return &list->head->data;
}

int ll_save(const LinkedList *list, const char *path)
{
    /*
     * Serialize entire list into a stack buffer and write via hal_storage
     * in a single batched transaction to minimize flash wear.
     */
    char buf[4096];
    size_t offset = 0;
    LinkedListNode *cur;

    if (!list || !path)
        return -1;

    buf[0] = '\0';
    for (cur = list->head; cur; cur = cur->next) {
        const Alert *a = &cur->data;
        int n = snprintf(buf + offset, sizeof(buf) - offset,
                         "%d %d %d %.6f %.8f %.8f %d %.6f\n",
                         a->id, (int)a->type, (int)a->severity,
                         a->timestamp, a->lat, a->lon,
                         a->retry_count, a->next_retry_time);
        if (n < 0 || offset + (size_t)n >= sizeof(buf))
            break;
        offset += (size_t)n;
    }
    return hal_storage_write_file(path, buf, offset);
}

int ll_load(LinkedList *list, const char *path)
{
    char buf[4096];
    size_t len = 0;
    const char *p;
    Alert a;
    int t, s;

    if (!list || !path)
        return -1;
    ll_clear(list);

    if (hal_storage_read_file(path, buf, sizeof(buf), &len) != 0 || len == 0)
        return 0; /* no file yet — empty outbox is fine */

    p = buf;
    while (*p) {
        int consumed = 0;
        if (sscanf(p, "%d %d %d %lf %lf %lf %d %lf%n",
                   &a.id, &t, &s,
                   &a.timestamp, &a.lat, &a.lon,
                   &a.retry_count, &a.next_retry_time, &consumed) == 8) {
            a.type = (AlertType)t;
            a.severity = (Severity)s;
            if (ll_append(list, &a) != 0)
                return -1;
            p += consumed;
            while (*p == '\n' || *p == '\r' || *p == ' ')
                p++;
        } else {
            break;
        }
    }
    return 0;
}
