/**
 * modem_at.h — SIM7670C 4G LTE + GNSS AT-Command Protocol Driver.
 *
 * Implements:
 *   - Network registration & packet domain attach (AT, AT+CPIN?, AT+CSQ, AT+CREG?, AT+CGATT=1)
 *   - GNSS fix acquisition & coordinate parser (AT+CGNSSPWR=1, AT+CGNSSINFO)
 *   - HTTPS POST alert dispatch with HMAC-SHA256 device signature, retries, and timeouts
 *
 * Pure C11 — runs identically on desktop `sim` HAL and ESP32 UART2 HAL.
 */
#ifndef MODEM_AT_H
#define MODEM_AT_H

#include "config.h"
#include "hal.h"
#include "security.h"
#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int    attached;
    int    csq;
    int    gnss_valid;
    double last_lat;
    double last_lon;
    int    at_retries;
    int    https_posts_ok;
    int    https_posts_err;
} ModemAtDriver;

void modem_at_init(ModemAtDriver *drv);

/** Execute full AT network registration sequence with retries. */
int  modem_at_attach(ModemAtDriver *drv, int max_retries);

/** Query AT+CGNSSINFO and parse latitude/longitude fix. */
int  modem_at_poll_gnss(ModemAtDriver *drv, double *lat, double *lon);

/**
 * Perform authenticated HTTPS POST of an Alert using SIM7670C HTTP stack:
 *   AT+HTTPINIT -> AT+HTTPPARA="URL" -> AT+HTTPDATA -> AT+HTTPACTION=1 -> AT+HTTPTERM
 */
int  modem_at_https_post_alert(ModemAtDriver *drv, const SecurityCtx *sec,
                               const Alert *alert);

#ifdef __cplusplus
}
#endif

#endif /* MODEM_AT_H */
