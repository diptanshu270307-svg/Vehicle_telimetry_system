/**
 * test_benchmark.c — Micro-benchmark harness measuring real latency (ns) and
 * CPU cycle counts for the core O(1) data structures and anomaly detector.
 */
#include "../src/config.h"
#include "../src/circular_queue.h"
#include "../src/linked_list.h"
#include "../src/detector.h"
#include "../src/security.h"
#include "../src/hal.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__)
static inline uint64_t read_cycles(void)
{
    uint32_t lo, hi;
    __asm__ __volatile__("rdtsc" : "=a"(lo), "=d"(hi));
    return ((uint64_t)hi << 32) | lo;
}
#define HAS_HW_CYCLES 1
#elif defined(__aarch64__)
static inline uint64_t read_cycles(void)
{
    uint64_t val;
    __asm__ __volatile__("mrs %0, cntvct_el0" : "=r"(val));
    return val;
}
#define HAS_HW_CYCLES 1
#else
static inline uint64_t read_cycles(void) { return 0; }
#define HAS_HW_CYCLES 0
#endif

static double monotonic_ns(void)
{
#ifdef _WIN32
    return (double)clock() * (1e9 / (double)CLOCKS_PER_SEC);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
#endif
}

int main(void)
{
    const int N = 100000;
    CircularQueue cq;
    Detector det;
    LinkedList ll;
    SecurityCtx sec;
    TelemetryPacket pkt;
    TelemetryPacket snap[CQ_DEFAULT_CAPACITY];
    Alert alert;
    char sig[SEC_SIG_HEX_LEN];
    double t0, t1, cq_ns, det_ns, snap_ns, ll_ns, hmac_ns;
    uint64_t c0, c1, cq_cyc, det_cyc;
    int i;

    printf("test_benchmark (N=%d iterations)\n", N);

    cq_init(&cq, CQ_DEFAULT_CAPACITY);
    detector_init(&det);
    ll_init(&ll);
    sec_init(&sec, CFG_DEVICE_ID, "benchmark_secret_key");

    memset(&pkt, 0, sizeof(pkt));
    pkt.vehicle_id  = 1;
    pkt.speed       = 60.0;
    pkt.accel_x     = 0.15;
    pkt.accel_y     = -0.05;
    pkt.accel_z     = CFG_GRAVITY_MPS2;
    pkt.engine_temp = 88.0;
    pkt.gps_valid   = 1;

    /* 1. Benchmark cq_enqueue O(1) */
    t0 = monotonic_ns();
    c0 = read_cycles();
    for (i = 0; i < N; i++) {
        pkt.timestamp = i * 0.004;
        cq_enqueue(&cq, &pkt);
    }
    c1 = read_cycles();
    t1 = monotonic_ns();
    cq_ns  = (t1 - t0) / (double)N;
    cq_cyc = (c1 - c0) / (uint64_t)N;

    /* 2. Benchmark detector_process O(1) */
    t0 = monotonic_ns();
    c0 = read_cycles();
    for (i = 0; i < N; i++) {
        pkt.timestamp = i * 0.004;
        pkt.accel_x   = (i & 1) ? 0.2 : -0.2;
        detector_process(&det, &pkt, &alert);
    }
    c1 = read_cycles();
    t1 = monotonic_ns();
    det_ns  = (t1 - t0) / (double)N;
    det_cyc = (c1 - c0) / (uint64_t)N;

    /* 3. Benchmark cq_snapshot O(k) for k=128 */
    t0 = monotonic_ns();
    for (i = 0; i < 10000; i++) {
        cq_snapshot(&cq, snap, CQ_DEFAULT_CAPACITY);
    }
    t1 = monotonic_ns();
    snap_ns = (t1 - t0) / 10000.0;

    /* 4. Benchmark static-pool ll_insert_by_severity + ll_remove_by_id */
    memset(&alert, 0, sizeof(alert));
    alert.id = 1;
    alert.type = ALERT_COLLISION;
    alert.severity = SEV_CRITICAL;
    t0 = monotonic_ns();
    for (i = 0; i < N; i++) {
        alert.id = (i & 15) + 1;
        ll_insert_by_severity(&ll, &alert);
        ll_remove_by_id(&ll, alert.id);
    }
    t1 = monotonic_ns();
    ll_ns = (t1 - t0) / (double)N;

    /* 5. Benchmark HMAC-SHA256 alert signing */
    t0 = monotonic_ns();
    for (i = 0; i < 10000; i++) {
        sec_sign_alert(&sec, &alert, sig, sizeof(sig));
    }
    t1 = monotonic_ns();
    hmac_ns = (t1 - t0) / 10000.0;

    printf("  +----------------------------------------+-------------+-------------+\n");
    printf("  | Operation                              | Latency(ns) | Counter/Cyc |\n");
    printf("  +----------------------------------------+-------------+-------------+\n");
    printf("  | cq_enqueue (O(1) static ring buffer)   | %9.2f ns | %9llu   |\n",
           cq_ns, (unsigned long long)cq_cyc);
    printf("  | detector_process (O(1) 16-win fusion)  | %9.2f ns | %9llu   |\n",
           det_ns, (unsigned long long)det_cyc);
    printf("  | cq_snapshot (O(k) k=128 packets)       | %9.2f ns |       -     |\n",
           snap_ns);
    printf("  | ll_insert+remove (static pool O(1))    | %9.2f ns |       -     |\n",
           ll_ns);
    printf("  | sec_sign_alert (HMAC-SHA256)           | %9.2f ns |       -     |\n",
           hmac_ns);
    printf("  +----------------------------------------+-------------+-------------+\n");

    /* Assert sub-100us real-time budget (250 Hz tick budget is 4,000,000 ns) */
    assert(det_ns < 100000.0);
    assert(cq_ns  < 100000.0);
    printf("  OK 250 Hz real-time budget verified (%.2f ns << 4,000,000 ns budget)\n",
           det_ns + cq_ns);
    printf("ALL PASSED\n");
    return 0;
}
