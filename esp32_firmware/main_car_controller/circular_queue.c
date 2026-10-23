/**
 * circular_queue.c — Malloc-free static ring-buffer black box implementation.
 *
 * WHERE USED: main loop enqueues every reading (O(1)); on anomaly,
 * detector/main calls cq_snapshot to reconstruct the pre-crash timeline.
 * Zero dynamic allocation (safe for deterministic embedded RTOS tasks).
 */
#include "circular_queue.h"
#include <string.h>

int cq_init(CircularQueue *cq, size_t capacity)
{
    if (!cq || capacity == 0 || capacity > CQ_MAX_CAPACITY)
        return -1;
    memset(cq->storage, 0, sizeof(cq->storage));
    cq->buf        = cq->storage;
    cq->capacity   = capacity;
    cq->head       = 0;
    cq->tail       = 0;
    cq->count      = 0;
    cq->overwrites = 0;
    return 0;
}

void cq_free(CircularQueue *cq)
{
    if (!cq)
        return;
    cq->buf = NULL;
    cq->capacity = cq->head = cq->tail = cq->count = cq->overwrites = 0;
}

void cq_enqueue(CircularQueue *cq, const TelemetryPacket *pkt)
{
    /* O(1): write at tail, advance with wraparound. */
    if (!cq || !pkt || !cq->buf)
        return;

    cq->buf[cq->tail] = *pkt;
    cq->tail = (cq->tail + 1) % cq->capacity;

    if (cq->count == cq->capacity) {
        /* Full -> overwrite oldest: advance head (black-box semantics). */
        cq->head = (cq->head + 1) % cq->capacity;
        cq->overwrites++;
    } else {
        cq->count++;
    }
}

int cq_dequeue(CircularQueue *cq, TelemetryPacket *out)
{
    if (!cq || cq_is_empty(cq))
        return -1;
    if (out)
        *out = cq->buf[cq->head];
    cq->head = (cq->head + 1) % cq->capacity;
    cq->count--;
    return 0;
}

int cq_peek(const CircularQueue *cq, TelemetryPacket *out)
{
    if (!cq || cq_is_empty(cq) || !out)
        return -1;
    *out = cq->buf[cq->head];
    return 0;
}

int cq_is_full(const CircularQueue *cq)
{
    return cq && cq->count == cq->capacity;
}

int cq_is_empty(const CircularQueue *cq)
{
    return !cq || cq->count == 0;
}

size_t cq_count(const CircularQueue *cq)
{
    return cq ? cq->count : 0;
}

size_t cq_snapshot(const CircularQueue *cq, TelemetryPacket *out, size_t max_out)
{
    /* O(k): walk from head chronologically with modular indices. */
    size_t i, n;
    if (!cq || !out || max_out == 0 || cq_is_empty(cq))
        return 0;
    n = cq->count < max_out ? cq->count : max_out;
    for (i = 0; i < n; i++) {
        size_t idx = (cq->head + i) % cq->capacity;
        out[i] = cq->buf[idx];
    }
    return n;
}
