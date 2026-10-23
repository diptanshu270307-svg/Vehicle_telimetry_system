/**
 * modem_at.c — SIM7670C 4G LTE + GNSS AT-Command Protocol Driver.
 *
 * Pure C11 with zero hardware #ifdefs. Uses hal_modem_send_at() and sec_sign_alert().
 */
#include "modem_at.h"
#include <stdio.h>
#include <string.h>

void modem_at_init(ModemAtDriver *drv)
{
    if (!drv)
        return;
    memset(drv, 0, sizeof(*drv));
    hal_modem_init();
}

static int send_at_with_retry(ModemAtDriver *drv, const char *cmd,
                              char *resp, size_t resp_len,
                              int timeout_ms, int max_retries)
{
    int attempt;
    for (attempt = 0; attempt <= max_retries; attempt++) {
        if (hal_modem_send_at(cmd, resp, resp_len, timeout_ms) == 0) {
            return 0;
        }
        if (drv)
            drv->at_retries++;
    }
    return -1;
}

int modem_at_attach(ModemAtDriver *drv, int max_retries)
{
    char resp[128];
    if (!drv)
        return -1;

    if (send_at_with_retry(drv, "AT", resp, sizeof(resp), 200, max_retries) != 0)
        return -1;
    if (send_at_with_retry(drv, "AT+CPIN?", resp, sizeof(resp), 300, max_retries) != 0)
        return -1;
    if (send_at_with_retry(drv, "AT+CSQ", resp, sizeof(resp), 300, max_retries) == 0) {
        int csq = 0;
        if (sscanf(resp, "+CSQ: %d", &csq) == 1)
            drv->csq = csq;
    }
    if (send_at_with_retry(drv, "AT+CREG?", resp, sizeof(resp), 500, max_retries) != 0)
        return -1;
    if (send_at_with_retry(drv, "AT+CGATT=1", resp, sizeof(resp), 1000, max_retries) != 0)
        return -1;
    if (send_at_with_retry(drv, "AT+CGNSSPWR=1", resp, sizeof(resp), 500, max_retries) != 0)
        return -1;

    drv->attached = 1;
    return 0;
}

int modem_at_poll_gnss(ModemAtDriver *drv, double *lat, double *lon)
{
    char resp[160];
    double parsed_lat = 0.0, parsed_lon = 0.0;
    int fix_mode = 0, sats = 0;
    char ns = 'N', ew = 'E';

    if (!drv)
        return -1;

    if (hal_modem_send_at("AT+CGNSSINFO", resp, sizeof(resp), 300) != 0) {
        drv->gnss_valid = 0;
        return -1;
    }

    /* Parse format: +CGNSSINFO: 3,09,28.613900,N,77.209000,E */
    if (sscanf(resp, "+CGNSSINFO: %d,%d,%lf,%c,%lf,%c",
               &fix_mode, &sats, &parsed_lat, &ns, &parsed_lon, &ew) == 6 &&
        fix_mode > 0) {
        if (ns == 'S') parsed_lat = -parsed_lat;
        if (ew == 'W') parsed_lon = -parsed_lon;
        drv->gnss_valid = 1;
        drv->last_lat   = parsed_lat;
        drv->last_lon   = parsed_lon;
        if (lat) *lat = parsed_lat;
        if (lon) *lon = parsed_lon;
        return 0;
    }

    drv->gnss_valid = 0;
    return -1;
}

int modem_at_https_post_alert(ModemAtDriver *drv, const SecurityCtx *sec,
                              const Alert *alert)
{
    char cmd[256], resp[128], sig[SEC_SIG_HEX_LEN];

    if (!drv || !alert)
        return -1;

    sig[0] = '\0';
    if (sec)
        sec_sign_alert(sec, alert, sig, sizeof(sig));

    if (!hal_modem_is_link_up()) {
        drv->https_posts_err++;
        hal_modem_publish_alert(alert, sig);
        return -1;
    }

    /* 1. Initialize HTTP service */
    if (send_at_with_retry(drv, "AT+HTTPINIT", resp, sizeof(resp), 300, 1) != 0) {
        drv->https_posts_err++;
        return -1;
    }

    /* 2. Configure endpoint URL from config.h */
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", CFG_CLOUD_ENDPOINT_URL);
    if (send_at_with_retry(drv, cmd, resp, sizeof(resp), 300, 1) != 0) {
        hal_modem_send_at("AT+HTTPTERM", resp, sizeof(resp), 200);
        drv->https_posts_err++;
        return -1;
    }

    /* 3. Set JSON content-type & execute POST via HAL */
    if (hal_modem_publish_alert(alert, sig) != 0) {
        hal_modem_send_at("AT+HTTPTERM", resp, sizeof(resp), 200);
        drv->https_posts_err++;
        return -1;
    }

    hal_modem_send_at("AT+HTTPTERM", resp, sizeof(resp), 200);
    drv->https_posts_ok++;
    return 0;
}
