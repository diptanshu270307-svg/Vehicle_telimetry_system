/**
 * security.h — Device Authentication, HMAC-SHA256 Signing & Anti-Replay Verification.
 *
 * Security Invariants (Phase 5):
 *   1. Per-device credential (`device_id` + pre-shared secret key loaded from
 *      environment or provisioned NVS; placeholder used in hardware-free demo).
 *   2. Outgoing alerts are signed with HMAC-SHA256 so the cloud backend can
 *      verify telemetry authenticity and integrity.
 *   3. Incoming remote commands (`LOCKDOWN`, `DISARM_OVERRIDE`, `ARM`) MUST
 *      include a fresh `timestamp`, a unique 64-bit `nonce`, and a valid
 *      HMAC-SHA256 `signature`.
 *   4. Commands outside `CFG_CMD_MAX_SKEW_SEC` or reusing a previously seen
 *      nonce are rejected to prevent replay attacks.
 *   5. No bare "unlock" endpoint exists anywhere in the system.
 */
#ifndef SECURITY_H
#define SECURITY_H

#include "config.h"
#include "structs.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SEC_NONCE_CACHE_SIZE 32
#define SEC_SIG_HEX_LEN      65 /* 64 hex chars + NUL */

typedef struct {
    char     cmd[32];               /* e.g. "DISARM", "LOCKDOWN", "ARM" */
    char     pin[16];               /* optional operator PIN            */
    double   timestamp;             /* command issue time (seconds)     */
    uint64_t nonce;                 /* unique per-request nonce         */
    char     signature[SEC_SIG_HEX_LEN];
} SignedCommand;

typedef struct {
    char     device_id[32];
    char     psk[64];
    uint64_t used_nonces[SEC_NONCE_CACHE_SIZE];
    size_t   nonce_head;
    size_t   nonce_count;
    int      verified_count;
    int      rejected_count;
    char     last_audit_log[128];
} SecurityCtx;

/** Initialize security context (reads TARE_DEVICE_PSK env if available). */
void sec_init(SecurityCtx *ctx, const char *device_id, const char *psk);

/** Compute HMAC-SHA256 hex digest over arbitrary message string. */
void sec_hmac_sha256_hex(const char *key, const char *message,
                         char *out_hex, size_t out_len);

/** Sign an outgoing Alert payload for SIM7670C / Cloud verification. */
void sec_sign_alert(const SecurityCtx *ctx, const Alert *alert,
                    char *sig_hex, size_t sig_len);

/** Sign a remote command structure (used by mock cloud / tests / PWA). */
void sec_sign_command(const SecurityCtx *ctx, SignedCommand *scmd);

/**
 * Verify a signed remote command against timestamp skew, nonce replay cache,
 * and constant-time HMAC-SHA256 comparison.
 * Returns 1 if authentic and fresh, 0 if rejected (reason written to err_out).
 */
int  sec_verify_command(SecurityCtx *ctx, const SignedCommand *scmd,
                        double now_sec, char *err_out, size_t err_len);

#ifdef __cplusplus
}
#endif

#endif /* SECURITY_H */
