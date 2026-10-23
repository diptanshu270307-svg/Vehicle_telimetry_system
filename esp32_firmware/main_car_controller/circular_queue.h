/**
 * circular_queue.h — Fixed-size static ring buffer ("black box") of TelemetryPacket.
 *
 * DATA STRUCTURE: Circular Queue / Ring Buffer (100% Malloc-Free Static Storage)
 *   - Contiguous static array `storage[CFG_CQ_MAX_CAPACITY]`.
 *   - head = next dequeue index, tail = next enqueue index, count = occupancy.
 *   - When full, enqueue OVERWRITES the oldest entry (black-box semantics).
 *   - Zero dynamic memory allocation (no malloc/calloc/free).
 *
 * TIME COMPLEXITY (all O(1) unless noted):
 *   cq_init, cq_enqueue, cq_dequeue, cq_peek,
 *   cq_is_full, cq_is_empty, cq_count  -> O(1)
 *   cq_snapshot                        -> O(k) where k = number of packets copied
 */
#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include "config.h"
#include "structs.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CQ_DEFAULT_CAPACITY CFG_CQ_DEFAULT_CAPACITY
#define CQ_MAX_CAPACITY     CFG_CQ_MAX_CAPACITY

typedef struct {
    TelemetryPacket storage[CQ_MAX_CAPACITY]; /* static pool — zero malloc    */
    TelemetryPacket *buf; /* points to storage[] for API compatibility        */
    size_t capacity;      /* active max packets held (<= CQ_MAX_CAPACITY)     */
    size_t head;          /* index of oldest packet                           */
    size_t tail;          /* index where next packet will be written          */
    size_t count;         /* current occupancy [0 .. capacity]                */
    size_t overwrites;    /* total ring overwrites (visualized on dashboard)  */
} CircularQueue;

/**
 * Initialize static ring buffer with given capacity (<= CQ_MAX_CAPACITY).
 * Returns 0 on success, -1 if capacity == 0 or > CQ_MAX_CAPACITY.
 * Complexity: O(1) — no heap allocation.
 */
int  cq_init(CircularQueue *cq, size_t capacity);

/** Reset indices (no heap free required). Complexity: O(1). */
void cq_free(CircularQueue *cq);

/**
 * Enqueue pkt. If full, overwrite oldest (advance head). Always succeeds.
 * Complexity: O(1) — single index write + modular arithmetic.
 */
void cq_enqueue(CircularQueue *cq, const TelemetryPacket *pkt);

/**
 * Remove oldest into *out. Returns 0 on success, -1 if empty.
 * Complexity: O(1).
 */
int  cq_dequeue(CircularQueue *cq, TelemetryPacket *out);

/**
 * Peek oldest without removing. Returns 0 on success, -1 if empty.
 * Complexity: O(1).
 */
int  cq_peek(const CircularQueue *cq, TelemetryPacket *out);

/** Complexity: O(1). */
int  cq_is_full(const CircularQueue *cq);
int  cq_is_empty(const CircularQueue *cq);
size_t cq_count(const CircularQueue *cq);

/**
 * Copy up to max_out packets in chronological order (oldest->newest) into out[].
 * Returns number copied. Used for pre-crash window reconstruction.
 * Complexity: O(k) where k = min(count, max_out).
 */
size_t cq_snapshot(const CircularQueue *cq, TelemetryPacket *out, size_t max_out);

#ifdef __cplusplus
}
#endif

#endif /* CIRCULAR_QUEUE_H */
