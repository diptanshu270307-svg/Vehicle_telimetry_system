/**
 * test_circular_queue.c — assert-based unit tests for the ring-buffer black box.
 * Covers: wraparound, overflow overwrite, empty/full, snapshot order.
 */
#include "../src/circular_queue.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static TelemetryPacket make_pkt(double t)
{
    TelemetryPacket p;
    memset(&p, 0, sizeof(p));
    p.timestamp = t;
    p.vehicle_id = 1;
    p.speed = t * 10.0;
    p.latitude = 1.0 + t;
    p.longitude = 2.0 + t;
    return p;
}

static void test_empty_full(void)
{
    CircularQueue cq;
    TelemetryPacket p;
    assert(cq_init(&cq, 3) == 0);
    assert(cq_is_empty(&cq));
    assert(!cq_is_full(&cq));
    assert(cq_count(&cq) == 0);
    assert(cq_dequeue(&cq, &p) == -1);
    assert(cq_peek(&cq, &p) == -1);

    p = make_pkt(1);
    cq_enqueue(&cq, &p);
    assert(!cq_is_empty(&cq));
    assert(!cq_is_full(&cq));
    assert(cq_count(&cq) == 1);

    p = make_pkt(2); cq_enqueue(&cq, &p);
    p = make_pkt(3); cq_enqueue(&cq, &p);
    assert(cq_is_full(&cq));
    assert(cq_count(&cq) == 3);

    cq_free(&cq);
    printf("  OK empty/full\n");
}

static void test_wraparound_and_overwrite(void)
{
    CircularQueue cq;
    TelemetryPacket p, out;
    TelemetryPacket snap[8];
    size_t n;

    assert(cq_init(&cq, 3) == 0);

    p = make_pkt(1); cq_enqueue(&cq, &p);
    p = make_pkt(2); cq_enqueue(&cq, &p);
    p = make_pkt(3); cq_enqueue(&cq, &p);
    /* Full: [1,2,3]. Next enqueue overwrites oldest (1). */
    p = make_pkt(4); cq_enqueue(&cq, &p);
    assert(cq_is_full(&cq));
    assert(cq_count(&cq) == 3);

    assert(cq_peek(&cq, &out) == 0);
    assert(out.timestamp == 2.0); /* oldest is now 2 */

    assert(cq_dequeue(&cq, &out) == 0);
    assert(out.timestamp == 2.0);
    assert(cq_dequeue(&cq, &out) == 0);
    assert(out.timestamp == 3.0);
    assert(cq_dequeue(&cq, &out) == 0);
    assert(out.timestamp == 4.0);
    assert(cq_is_empty(&cq));

    /* Fill again to exercise wrap indices, then snapshot. */
    p = make_pkt(10); cq_enqueue(&cq, &p);
    p = make_pkt(11); cq_enqueue(&cq, &p);
    p = make_pkt(12); cq_enqueue(&cq, &p);
    p = make_pkt(13); cq_enqueue(&cq, &p); /* overwrite 10 */
    n = cq_snapshot(&cq, snap, 8);
    assert(n == 3);
    assert(snap[0].timestamp == 11.0);
    assert(snap[1].timestamp == 12.0);
    assert(snap[2].timestamp == 13.0);

    cq_free(&cq);
    printf("  OK wraparound / overwrite / snapshot\n");
}

int main(void)
{
    printf("test_circular_queue\n");
    test_empty_full();
    test_wraparound_and_overwrite();
    printf("ALL PASSED\n");
    return 0;
}

