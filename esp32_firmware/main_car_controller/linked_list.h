/**
 * linked_list.h — Singly-linked list for the alert outbox / event log.
 *
 * DATA STRUCTURE: Singly Linked List backed by an O(1) Static Node Pool.
 *   Each node holds one Alert. head -> ... -> tail -> NULL.
 *   Nodes are allocated from `pool[CFG_LL_POOL_SIZE]` via an O(1) free-list,
 *   making the entire engine 100% malloc-free and deterministic.
 *
 * TIME COMPLEXITY:
 *   ll_init / ll_is_empty / ll_count (cached) -> O(1)
 *   Node pool alloc / release                 -> O(1) via free_head
 *   ll_append                                 -> O(1) with tail pointer
 *   ll_insert_by_severity                     -> O(n) scan to find insert point
 *   ll_remove_by_id / ll_find                 -> O(n)
 *   ll_clear                                  -> O(n)
 *   ll_save / ll_load                         -> O(n) via hal_storage
 */
#ifndef LINKED_LIST_H
#define LINKED_LIST_H

#include "config.h"
#include "structs.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LL_POOL_SIZE CFG_LL_POOL_SIZE

typedef struct LinkedListNode {
    Alert data;
    struct LinkedListNode *next;
} LinkedListNode;

typedef struct {
    LinkedListNode *head;
    LinkedListNode *tail;                  /* for O(1) append                 */
    LinkedListNode *free_head;             /* O(1) free-list of unused nodes  */
    LinkedListNode  pool[LL_POOL_SIZE];    /* static node storage (no malloc) */
    size_t count;
} LinkedList;

void ll_init(LinkedList *list);
void ll_clear(LinkedList *list); /* return all nodes to static free-list */

int  ll_is_empty(const LinkedList *list);
size_t ll_count(const LinkedList *list);
size_t ll_pool_free_count(const LinkedList *list);

/** Append at tail. O(1). Returns 0 on success, -1 if static pool exhausted. */
int  ll_append(LinkedList *list, const Alert *alert);

/**
 * Insert so that higher severity comes first (stable among equals: append
 * after last equal-severity node). O(n). Returns 0 / -1.
 */
int  ll_insert_by_severity(LinkedList *list, const Alert *alert);

/** Remove first node with matching id. O(n). Returns 0 if found, -1 else. */
int  ll_remove_by_id(LinkedList *list, int id);

/** Find by id. O(n). Returns pointer into list (do not free), or NULL. */
Alert *ll_find(LinkedList *list, int id);

/** Peek head alert (highest priority after severity inserts). O(1). */
Alert *ll_peek_head(LinkedList *list);

/**
 * Persist entire list via hal_storage (one alert per line).
 * Reloaded on startup so outbox survives crash/restart. O(n).
 */
int  ll_save(const LinkedList *list, const char *path);
int  ll_load(LinkedList *list, const char *path);

#ifdef __cplusplus
}
#endif

#endif /* LINKED_LIST_H */
