/**
 * test_linked_list.c — insert, delete, severity ordering, static node pool,
 * and batched LittleFS/disk persistence.
 */
#include "../src/linked_list.h"
#include "../src/publisher.h"
#include "../src/hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static Alert make_alert(int id, Severity sev)
{
    Alert a;
    memset(&a, 0, sizeof(a));
    a.id = id;
    a.type = ALERT_ANOMALY;
    a.severity = sev;
    a.timestamp = (double)id;
    a.lat = 10.0 + id;
    a.lon = 20.0 + id;
    return a;
}

static void test_append_remove_and_static_pool(void)
{
    LinkedList list;
    Alert a;
    size_t i;

    ll_init(&list);
    assert(ll_is_empty(&list));
    assert(ll_pool_free_count(&list) == LL_POOL_SIZE);

    a = make_alert(1, SEV_LOW);
    assert(ll_append(&list, &a) == 0);
    a = make_alert(2, SEV_LOW);
    assert(ll_append(&list, &a) == 0);
    assert(ll_count(&list) == 2);
    assert(ll_pool_free_count(&list) == LL_POOL_SIZE - 2);
    assert(ll_find(&list, 2) != NULL);
    assert(ll_remove_by_id(&list, 1) == 0);
    assert(ll_count(&list) == 1);
    assert(ll_pool_free_count(&list) == LL_POOL_SIZE - 1);
    assert(ll_find(&list, 1) == NULL);
    assert(ll_remove_by_id(&list, 99) == -1);
    ll_clear(&list);
    assert(ll_is_empty(&list));
    assert(ll_pool_free_count(&list) == LL_POOL_SIZE);

    /* Fill entire static pool to verify graceful exhaustion without malloc */
    for (i = 0; i < LL_POOL_SIZE; i++) {
        a = make_alert((int)i + 100, SEV_MEDIUM);
        assert(ll_append(&list, &a) == 0);
    }
    assert(ll_pool_free_count(&list) == 0);
    a = make_alert(999, SEV_HIGH);
    assert(ll_append(&list, &a) == -1); /* pool full -> returns -1 safely */
    assert(ll_remove_by_id(&list, 100) == 0);
    assert(ll_append(&list, &a) == 0);  /* freed slot reused in O(1) */
    ll_clear(&list);

    printf("  OK append/delete + malloc-free static node pool (%d slots)\n", LL_POOL_SIZE);
}

static void test_severity_order(void)
{
    LinkedList list;
    Alert a;
    LinkedListNode *cur;
    int prev_sev;

    ll_init(&list);
    a = make_alert(1, SEV_LOW);      ll_insert_by_severity(&list, &a);
    a = make_alert(2, SEV_CRITICAL); ll_insert_by_severity(&list, &a);
    a = make_alert(3, SEV_HIGH);     ll_insert_by_severity(&list, &a);
    a = make_alert(4, SEV_MEDIUM);   ll_insert_by_severity(&list, &a);
    a = make_alert(5, SEV_CRITICAL); ll_insert_by_severity(&list, &a);

    assert(ll_count(&list) == 5);
    assert(ll_peek_head(&list)->severity == SEV_CRITICAL);

    prev_sev = 999;
    for (cur = list.head; cur; cur = cur->next) {
        assert((int)cur->data.severity <= prev_sev);
        prev_sev = (int)cur->data.severity;
    }
    assert(list.head->data.id == 2);
    assert(list.head->next->data.id == 5);

    ll_clear(&list);
    printf("  OK severity ordering\n");
}

static void test_persistence_and_batching(void)
{
    LinkedList a, b;
    Publisher pub;
    Alert x;
    const char *path = "data/test_outbox.txt";
    int i;

    hal_storage_init();

    ll_init(&a);
    x = make_alert(10, SEV_HIGH);     ll_insert_by_severity(&a, &x);
    x = make_alert(11, SEV_CRITICAL); ll_insert_by_severity(&a, &x);
    x = make_alert(12, SEV_LOW);      ll_insert_by_severity(&a, &x);
    assert(ll_save(&a, path) == 0);

    ll_init(&b);
    assert(ll_load(&b, path) == 0);
    assert(ll_count(&b) == 3);
    assert(ll_peek_head(&b)->id == 11);
    assert(ll_find(&b, 12) != NULL);
    assert(ll_find(&b, 12)->severity == SEV_LOW);

    ll_clear(&a);
    ll_clear(&b);
    remove(path);

    /* Verify flash-wear batching in Publisher: 10 rapid enqueues -> 1 flash write */
    remove(PUB_OUTBOX_PATH);
    publisher_init(&pub);
    publisher_set_network(&pub, 0);
    for (i = 0; i < 10; i++) {
        x = make_alert(200 + i, SEV_HIGH);
        x.next_retry_time = 100.0;
        publisher_enqueue(&pub, &x);
    }
    assert(pub.mutations_count == 10);
    assert(pub.flash_writes == 0); /* still batched in RAM */

    publisher_tick(&pub, 2.5);     /* >= CFG_STORAGE_BATCH_INTERVAL -> 1 flush */
    assert(pub.flash_writes == 1);
    publisher_shutdown(&pub);
    remove(PUB_OUTBOX_PATH);

    printf("  OK save/load + batched flash-wear persistence (10 mutations -> 1 write)\n");
}

int main(void)
{
    printf("test_linked_list\n");
    test_append_remove_and_static_pool();
    test_severity_order();
    test_persistence_and_batching();
    printf("ALL PASSED\n");
    return 0;
}
