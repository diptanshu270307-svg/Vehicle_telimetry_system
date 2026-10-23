/**
 * test_security_modem.c — Unit tests for HMAC-SHA256 signing, anti-replay
 * remote command verification, and SIM7670C AT-command protocol driver.
 */
#include "../src/security.h"
#include "../src/modem_at.h"
#include "../src/hal.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void test_hmac_sha256_and_alert_signing(void)
{
    SecurityCtx sec;
    Alert a;
    char sig1[SEC_SIG_HEX_LEN], sig2[SEC_SIG_HEX_LEN];

    sec_init(&sec, "CAR-ESP32-042", "super_secret_test_key_2026");

    memset(&a, 0, sizeof(a));
    a.id = 7;
    a.type = ALERT_COLLISION;
    a.severity = SEV_CRITICAL;
    a.timestamp = 12.345;
    a.lat = 28.6139;
    a.lon = 77.2090;

    sec_sign_alert(&sec, &a, sig1, sizeof(sig1));
    assert(strlen(sig1) == 64);

    /* Tampering with alert coordinate changes signature completely */
    a.lat = 28.9999;
    sec_sign_alert(&sec, &a, sig2, sizeof(sig2));
    assert(strcmp(sig1, sig2) != 0);

    printf("  OK HMAC-SHA256 alert signing & tamper detection (sig=%.16s...)\n", sig1);
}

static void test_command_anti_replay_and_no_bare_unlock(void)
{
    SecurityCtx sec;
    SignedCommand cmd;
    char err[128];

    sec_init(&sec, "CAR-ESP32-042", "super_secret_test_key_2026");

    /* 1. Valid signed DISARM command */
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.cmd, sizeof(cmd.cmd), "DISARM");
    snprintf(cmd.pin, sizeof(cmd.pin), "1234");
    cmd.timestamp = 100.0;
    cmd.nonce     = 0x1001ULL;
    sec_sign_command(&sec, &cmd);

    assert(sec_verify_command(&sec, &cmd, 102.0, err, sizeof(err)) == 1);

    /* 2. Replay attack with exact same signed command + nonce -> REJECTED */
    assert(sec_verify_command(&sec, &cmd, 103.0, err, sizeof(err)) == 0);
    assert(strstr(err, "REJECTED_REPLAY") != NULL);

    /* 3. Stale timestamp (> 30s drift) -> REJECTED */
    cmd.nonce = 0x1002ULL;
    cmd.timestamp = 50.0;
    sec_sign_command(&sec, &cmd);
    assert(sec_verify_command(&sec, &cmd, 100.0, err, sizeof(err)) == 0);
    assert(strstr(err, "REJECTED_STALE") != NULL);

    /* 4. Tampered signature -> REJECTED */
    cmd.nonce = 0x1003ULL;
    cmd.timestamp = 100.0;
    sec_sign_command(&sec, &cmd);
    cmd.signature[0] = (cmd.signature[0] == 'a') ? 'b' : 'a';
    assert(sec_verify_command(&sec, &cmd, 100.0, err, sizeof(err)) == 0);
    assert(strstr(err, "REJECTED_SIG") != NULL);

    /* 5. Bare "UNLOCK" endpoint command -> REJECTED unconditionally */
    snprintf(cmd.cmd, sizeof(cmd.cmd), "UNLOCK");
    cmd.nonce = 0x1004ULL;
    cmd.timestamp = 100.0;
    sec_sign_command(&sec, &cmd);
    assert(sec_verify_command(&sec, &cmd, 100.0, err, sizeof(err)) == 0);
    assert(strstr(err, "UNLOCK") != NULL);

    printf("  OK command signature verification, nonce replay block, timestamp skew & bare UNLOCK block\n");
}

static void test_sim7670c_at_driver(void)
{
    ModemAtDriver drv;
    SecurityCtx sec;
    Alert a;
    double lat = 0.0, lon = 0.0;

    sec_init(&sec, CFG_DEVICE_ID, NULL);
    modem_at_init(&drv);

    assert(modem_at_attach(&drv, 2) == 0);
    assert(drv.attached == 1);
    assert(drv.csq > 0);

    assert(modem_at_poll_gnss(&drv, &lat, &lon) == 0);
    assert(drv.gnss_valid == 1);
    assert(fabs(lat - 28.6139) < 0.01);
    assert(fabs(lon - 77.2090) < 0.01);

    memset(&a, 0, sizeof(a));
    a.id = 1;
    a.type = ALERT_COLLISION;
    a.severity = SEV_CRITICAL;
    a.timestamp = 10.0;
    a.lat = lat;
    a.lon = lon;

    /* Network UP -> HTTPS POST succeeds */
    assert(modem_at_https_post_alert(&drv, &sec, &a) == 0);
    assert(drv.https_posts_ok == 1);

    /* Network DOWN -> HTTPS POST fails cleanly for outbox backoff */
    hal_modem_set_link_up(0);
    assert(modem_at_https_post_alert(&drv, &sec, &a) == -1);
    assert(drv.https_posts_err == 1);
    hal_modem_set_link_up(1);

    printf("  OK SIM7670C AT attach, CGNSSINFO fix parser, and HTTPS POST state machine\n");
}

int main(void)
{
    printf("test_security_modem\n");
    test_hmac_sha256_and_alert_signing();
    test_command_anti_replay_and_no_bare_unlock();
    test_sim7670c_at_driver();
    printf("ALL PASSED\n");
    return 0;
}
