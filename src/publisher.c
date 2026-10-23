/**
 * publisher.c — Resilient alert publisher + SIM7670C AT HTTPS + HMAC-SHA256 + LoRa fallback.
 *
 * Pure C11 with zero hardware #ifdefs.
 */
#include "publisher.h"
#include "hal.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static unsigned xorshift(unsigned *state)
{
    unsigned x = *state ? *state : 1u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static double rand01(Publisher *p)
{
    return (xorshift(&p->rng_state) & 0xFFFFFF) / (double)0x1000000;
}

void publisher_init(Publisher *p)
{
    if (!p)
        return;
    memset(p, 0, sizeof(*p));
    ll_init(&p->outbox);
    modem_at_init(&p->modem);
    modem_at_attach(&p->modem, 2);
    sec_init(&p->sec, CFG_DEVICE_ID, NULL);
    p->network_up = 1;
    p->force_disconnect = 0;
    p->flaky_rate = 0.25; /* 25% chance a publish attempt fails randomly */
    p->rng_state = 0xC0FFEEu;
    p->last_acked_id = -1;
    /* Reload durable outbox so alerts survive process restart. */
    ll_load(&p->outbox, PUB_OUTBOX_PATH);
}

int publisher_flush(Publisher *p, double now)
{
    if (!p)
        return -1;
    if (!p->dirty)
        return 0;
    if (ll_save(&p->outbox, PUB_OUTBOX_PATH) == 0) {
        p->dirty = 0;
        p->last_flush_time = now;
        p->flash_writes++;
        return 0;
    }
    return -1;
}

void publisher_shutdown(Publisher *p)
{
    if (!p)
        return;
    p->dirty = 1;
    publisher_flush(p, p->last_flush_time);
    ll_clear(&p->outbox);
}

int publisher_enqueue(Publisher *p, const Alert *alert)
{
    if (!p || !alert)
        return -1;
    /* Severity-ordered insert: CRITICAL drained first. O(n). */
    if (ll_insert_by_severity(&p->outbox, alert) != 0)
        return -1;
    sec_sign_alert(&p->sec, alert, p->last_alert_sig, sizeof(p->last_alert_sig));
    p->dirty = 1;
    p->mutations_count++;
    return 0;
}

static int try_send(Publisher *p, const Alert *a)
{
    int up = p->network_up && !p->force_disconnect;
    p->publish_attempts++;
    if (!up) {
        modem_at_https_post_alert(&p->modem, &p->sec, a);
        return 0;
    }
    /* Random disconnect / packet loss simulation. */
    if (rand01(p) < p->flaky_rate) {
        return 0;
    }
    if (modem_at_https_post_alert(&p->modem, &p->sec, a) != 0)
        return 0;
    p->publish_successes++;
    return 1; /* ACK */
}

void publisher_tick(Publisher *p, double now)
{
    LinkedListNode *cur;
    if (!p)
        return;
    cur = p->outbox.head;
    while (cur) {
        LinkedListNode *next = cur->next; /* save — node may be released */
        Alert *a = &cur->data;
        if (a->next_retry_time <= now) {
            if (try_send(p, a)) {
                int id = a->id;
                p->last_acked_id = id;
                printf("[PUB] ACK alert #%d (%s) @ t=%.2f [HMAC=%.12s...]\n",
                       id, alert_type_name(a->type), now, p->last_alert_sig);
                ll_remove_by_id(&p->outbox, id);
                p->dirty = 1;
                p->mutations_count++;
            } else {
                double backoff;
                a->retry_count++;
                backoff = pow(2.0, (double)(a->retry_count < 5 ? a->retry_count : 5));
                if (backoff > PUB_MAX_BACKOFF)
                    backoff = PUB_MAX_BACKOFF;
                a->next_retry_time = now + backoff;
                if (hal_lora_send_alert(a) == 0)
                    p->lora_fallbacks++;
                printf("[PUB] RETRY alert #%d in %.1fs (net=%s, LoRa fallback sent)\n",
                       a->id, backoff,
                       (p->network_up && !p->force_disconnect) ? "flaky" : "DOWN");
                p->dirty = 1;
                p->mutations_count++;
            }
        }
        cur = next;
    }

    /* Batched flash-wear-limiting write */
    if (p->dirty && (now - p->last_flush_time >= CFG_STORAGE_BATCH_INTERVAL)) {
        publisher_flush(p, now);
    }
}

void publisher_set_network(Publisher *p, int up)
{
    if (!p)
        return;
    p->force_disconnect = up ? 0 : 1;
    p->network_up = up ? 1 : 0;
    hal_modem_set_link_up(up);
    printf("[NET] Network %s\n", up ? "UP" : "DOWN");
}

void publisher_toggle_network(Publisher *p)
{
    if (!p)
        return;
    if (p->force_disconnect || !p->network_up)
        publisher_set_network(p, 1);
    else
        publisher_set_network(p, 0);
}

int publisher_network_up(const Publisher *p)
{
    if (!p)
        return 0;
    return p->network_up && !p->force_disconnect;
}

LinkedList *publisher_outbox(Publisher *p)
{
    return p ? &p->outbox : NULL;
}
