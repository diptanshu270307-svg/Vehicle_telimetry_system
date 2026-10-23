/**
 * publisher.h — Resilient alert publisher with batched persistence, SIM7670C AT
 * HTTPS POST, HMAC-SHA256 signing, and LoRa fallback.
 *
 * Outbox = LinkedList of Alert (static pool, malloc-free).
 * Retries with exponential backoff until ACK.
 * Batches LittleFS/disk writes (CFG_STORAGE_BATCH_INTERVAL) to limit flash wear.
 */
#ifndef PUBLISHER_H
#define PUBLISHER_H

#include "config.h"
#include "linked_list.h"
#include "modem_at.h"
#include "security.h"
#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PUB_OUTBOX_PATH CFG_OUTBOX_PATH
#define PUB_MAX_BACKOFF CFG_PUB_MAX_BACKOFF_SEC

typedef struct {
    LinkedList    outbox;
    ModemAtDriver modem;
    SecurityCtx   sec;
    int      network_up;       /* 1 = connected, 0 = disconnected          */
    int      force_disconnect; /* CLI/keyboard override                    */
    double   flaky_rate;       /* probability of random drop when "up"     */
    unsigned rng_state;
    int      last_acked_id;
    int      publish_attempts;
    int      publish_successes;
    int      lora_fallbacks;
    char     last_alert_sig[SEC_SIG_HEX_LEN];

    /* Flash wear batching */
    int      dirty;            /* 1 = outbox mutated since last disk flush */
    double   last_flush_time;  /* timestamp of last hal_storage flush      */
    int      mutations_count;  /* total outbox mutations                   */
    int      flash_writes;     /* actual batched file writes               */
} Publisher;

void publisher_init(Publisher *p);
void publisher_shutdown(Publisher *p); /* flush dirty outbox + clear pool */

/** Force immediate write of dirty outbox to storage. */
int  publisher_flush(Publisher *p, double now);

/** Enqueue alert (severity-ordered) and mark dirty for batched persistence. */
int  publisher_enqueue(Publisher *p, const Alert *alert);

/**
 * Attempt to publish due alerts at current time `now`, and flush dirty outbox
 * if CFG_STORAGE_BATCH_INTERVAL has elapsed.
 */
void publisher_tick(Publisher *p, double now);

void publisher_set_network(Publisher *p, int up);
void publisher_toggle_network(Publisher *p);
int  publisher_network_up(const Publisher *p);

LinkedList *publisher_outbox(Publisher *p);

#ifdef __cplusplus
}
#endif

#endif /* PUBLISHER_H */
