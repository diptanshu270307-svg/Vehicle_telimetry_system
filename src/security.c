/**
 * security.c — Pure C11 SHA-256, HMAC-SHA256, and Anti-Replay Command Verifier.
 *
 * Zero external dependencies, zero malloc.
 */
#include "security.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Compact Pure-C11 SHA-256 Implementation (FIPS 180-4) ---------------- */
typedef struct {
    uint32_t state[8];
    uint64_t bitcount;
    uint8_t  buffer[64];
} Sha256Ctx;

static const uint32_t K256[64] = {
    0x428a2f98UL,0x71374491UL,0xb5c0fbcfUL,0xe9b5dba5UL,
    0x3956c25bUL,0x59f111f1UL,0x923f82a4UL,0xab1c5ed5UL,
    0xd807aa98UL,0x12835b01UL,0x243185beUL,0x550c7dc3UL,
    0x72be5d74UL,0x80deb1feUL,0x9bdc06a7UL,0xc19bf174UL,
    0xe49b69c1UL,0xefbe4786UL,0x0fc19dc6UL,0x240ca1ccUL,
    0x2de92c6fUL,0x4a7484aaUL,0x5cb0a9dcUL,0x76f988daUL,
    0x983e5152UL,0xa831c66dUL,0xb00327c8UL,0xbf597fc7UL,
    0xc6e00bf3UL,0xd5a79147UL,0x06ca6351UL,0x14292967UL,
    0x27b70a85UL,0x2e1b2138UL,0x4d2c6dfcUL,0x53380d13UL,
    0x650a7354UL,0x766a0abbUL,0x81c2c92eUL,0x92722c85UL,
    0xa2bfe8a1UL,0xa81a664bUL,0xc24b8b70UL,0xc76c51a3UL,
    0xd192e819UL,0xd6990624UL,0xf40e3585UL,0x106aa070UL,
    0x19a4c116UL,0x1e376c08UL,0x2748774cUL,0x34b0bcb5UL,
    0x391c0cb3UL,0x4ed8aa4aUL,0x5b9cca4fUL,0x682e6ff3UL,
    0x748f82eeUL,0x78a5636fUL,0x84c87814UL,0x8cc70208UL,
    0x90befffaUL,0xa4506cebUL,0xbef9a3f7UL,0xc67178f2UL
};

static uint32_t rotr32(uint32_t x, unsigned n)
{
    return (x >> n) | (x << (32U - n));
}

static void sha256_transform(Sha256Ctx *ctx, const uint8_t data[64])
{
    uint32_t a, b, c, d, e, f, g, h, t1, t2, m[64];
    int i;

    for (i = 0; i < 16; i++) {
        m[i] = ((uint32_t)data[i * 4] << 24) |
               ((uint32_t)data[i * 4 + 1] << 16) |
               ((uint32_t)data[i * 4 + 2] << 8) |
               ((uint32_t)data[i * 4 + 3]);
    }
    for (; i < 64; i++) {
        uint32_t s0 = rotr32(m[i - 15], 7) ^ rotr32(m[i - 15], 18) ^ (m[i - 15] >> 3);
        uint32_t s1 = rotr32(m[i - 2], 17) ^ rotr32(m[i - 2], 19) ^ (m[i - 2] >> 10);
        m[i] = m[i - 16] + s0 + m[i - 7] + s1;
    }

    a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2]; d = ctx->state[3];
    e = ctx->state[4]; f = ctx->state[5]; g = ctx->state[6]; h = ctx->state[7];

    for (i = 0; i < 64; i++) {
        uint32_t S1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
        uint32_t ch = (e & f) ^ ((~e) & g);
        uint32_t S0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        t1 = h + S1 + ch + K256[i] + m[i];
        t2 = S0 + maj;
        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }

    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += d;
    ctx->state[4] += e; ctx->state[5] += f; ctx->state[6] += g; ctx->state[7] += h;
}

static void sha256_init(Sha256Ctx *ctx)
{
    ctx->state[0] = 0x6a09e667UL; ctx->state[1] = 0xbb67ae85UL;
    ctx->state[2] = 0x3c6ef372UL; ctx->state[3] = 0xa54ff53aUL;
    ctx->state[4] = 0x510e527fUL; ctx->state[5] = 0x9b05688cUL;
    ctx->state[6] = 0x1f83d9abUL; ctx->state[7] = 0x5be0cd19UL;
    ctx->bitcount = 0;
    memset(ctx->buffer, 0, sizeof(ctx->buffer));
}

static void sha256_update(Sha256Ctx *ctx, const uint8_t *data, size_t len)
{
    size_t i = 0;
    size_t idx = (size_t)((ctx->bitcount >> 3) & 0x3F);
    ctx->bitcount += (uint64_t)len << 3;

    for (i = 0; i < len; i++) {
        ctx->buffer[idx++] = data[i];
        if (idx == 64) {
            sha256_transform(ctx, ctx->buffer);
            idx = 0;
        }
    }
}

static void sha256_final(Sha256Ctx *ctx, uint8_t digest[32])
{
    size_t idx = (size_t)((ctx->bitcount >> 3) & 0x3F);
    int i;

    ctx->buffer[idx++] = 0x80;
    if (idx > 56) {
        while (idx < 64) ctx->buffer[idx++] = 0;
        sha256_transform(ctx, ctx->buffer);
        idx = 0;
    }
    while (idx < 56) ctx->buffer[idx++] = 0;

    for (i = 0; i < 8; i++) {
        ctx->buffer[63 - i] = (uint8_t)(ctx->bitcount >> (i * 8));
    }
    sha256_transform(ctx, ctx->buffer);

    for (i = 0; i < 8; i++) {
        digest[i * 4]     = (uint8_t)(ctx->state[i] >> 24);
        digest[i * 4 + 1] = (uint8_t)(ctx->state[i] >> 16);
        digest[i * 4 + 2] = (uint8_t)(ctx->state[i] >> 8);
        digest[i * 4 + 3] = (uint8_t)(ctx->state[i]);
    }
}

/* ---- HMAC-SHA256 (RFC 2104) ---------------------------------------------- */
void sec_hmac_sha256_hex(const char *key, const char *message,
                         char *out_hex, size_t out_len)
{
    uint8_t k_ipad[64], k_opad[64], tk[32], inner[32], outer[32];
    Sha256Ctx sctx;
    size_t klen, mlen, i;
    static const char hex[] = "0123456789abcdef";

    if (!key || !message || !out_hex || out_len < 65)
        return;

    klen = strlen(key);
    mlen = strlen(message);

    if (klen > 64) {
        sha256_init(&sctx);
        sha256_update(&sctx, (const uint8_t *)key, klen);
        sha256_final(&sctx, tk);
        key  = (const char *)tk;
        klen = 32;
    }

    memset(k_ipad, 0x36, 64);
    memset(k_opad, 0x5c, 64);
    for (i = 0; i < klen; i++) {
        k_ipad[i] ^= (uint8_t)key[i];
        k_opad[i] ^= (uint8_t)key[i];
    }

    sha256_init(&sctx);
    sha256_update(&sctx, k_ipad, 64);
    sha256_update(&sctx, (const uint8_t *)message, mlen);
    sha256_final(&sctx, inner);

    sha256_init(&sctx);
    sha256_update(&sctx, k_opad, 64);
    sha256_update(&sctx, inner, 32);
    sha256_final(&sctx, outer);

    for (i = 0; i < 32; i++) {
        out_hex[i * 2]     = hex[(outer[i] >> 4) & 0xF];
        out_hex[i * 2 + 1] = hex[outer[i] & 0xF];
    }
    out_hex[64] = '\0';
}

/* Constant-time string comparison to prevent timing side-channels */
static int const_time_eq(const char *a, const char *b, size_t len)
{
    uint8_t diff = 0;
    size_t i;
    for (i = 0; i < len; i++) {
        diff |= (uint8_t)(a[i] ^ b[i]);
    }
    return diff == 0;
}

void sec_init(SecurityCtx *ctx, const char *device_id, const char *psk)
{
    const char *env_psk;
    if (!ctx)
        return;
    memset(ctx, 0, sizeof(*ctx));
    snprintf(ctx->device_id, sizeof(ctx->device_id), "%s",
             device_id ? device_id : CFG_DEVICE_ID);

    env_psk = getenv("TARE_DEVICE_PSK");
    if (psk && psk[0]) {
        snprintf(ctx->psk, sizeof(ctx->psk), "%s", psk);
    } else if (env_psk && env_psk[0]) {
        snprintf(ctx->psk, sizeof(ctx->psk), "%s", env_psk);
    } else {
        snprintf(ctx->psk, sizeof(ctx->psk), "%s", CFG_DEVICE_PSK_PLACEHOLDER);
    }
    snprintf(ctx->last_audit_log, sizeof(ctx->last_audit_log),
             "SEC_INIT: device=%s HMAC-SHA256 active", ctx->device_id);
}

void sec_sign_alert(const SecurityCtx *ctx, const Alert *alert,
                    char *sig_hex, size_t sig_len)
{
    char payload[256];
    if (!ctx || !alert || !sig_hex)
        return;
    snprintf(payload, sizeof(payload),
             "dev=%s|id=%d|type=%d|sev=%d|t=%.3f|lat=%.6f|lon=%.6f",
             ctx->device_id, alert->id, (int)alert->type, (int)alert->severity,
             alert->timestamp, alert->lat, alert->lon);
    sec_hmac_sha256_hex(ctx->psk, payload, sig_hex, sig_len);
}

void sec_sign_command(const SecurityCtx *ctx, SignedCommand *scmd)
{
    char payload[256];
    if (!ctx || !scmd)
        return;
    snprintf(payload, sizeof(payload),
             "dev=%s|cmd=%s|pin=%s|t=%.3f|nonce=%llu",
             ctx->device_id, scmd->cmd, scmd->pin, scmd->timestamp,
             (unsigned long long)scmd->nonce);
    sec_hmac_sha256_hex(ctx->psk, payload, scmd->signature, sizeof(scmd->signature));
}

int sec_verify_command(SecurityCtx *ctx, const SignedCommand *scmd,
                       double now_sec, char *err_out, size_t err_len)
{
    char expected[SEC_SIG_HEX_LEN];
    char payload[256];
    size_t i;

    if (!ctx || !scmd)
        return 0;

    /* 1. Never allow bare "UNLOCK" command without cryptographic + PIN auth */
    if (strcmp(scmd->cmd, "UNLOCK") == 0) {
        ctx->rejected_count++;
        snprintf(ctx->last_audit_log, sizeof(ctx->last_audit_log),
                 "REJECTED: bare UNLOCK endpoint forbidden");
        if (err_out && err_len)
            snprintf(err_out, err_len, "%s", ctx->last_audit_log);
        return 0;
    }

    /* 2. Check timestamp freshness (anti-replay window) */
    if (fabs(now_sec - scmd->timestamp) > CFG_CMD_MAX_SKEW_SEC) {
        ctx->rejected_count++;
        snprintf(ctx->last_audit_log, sizeof(ctx->last_audit_log),
                 "REJECTED_STALE: timestamp skew %.1fs > %.1fs",
                 fabs(now_sec - scmd->timestamp), CFG_CMD_MAX_SKEW_SEC);
        if (err_out && err_len)
            snprintf(err_out, err_len, "%s", ctx->last_audit_log);
        return 0;
    }

    /* 3. Check nonce replay cache */
    if (scmd->nonce == 0) {
        ctx->rejected_count++;
        snprintf(ctx->last_audit_log, sizeof(ctx->last_audit_log),
                 "REJECTED_NONCE: zero nonce invalid");
        if (err_out && err_len)
            snprintf(err_out, err_len, "%s", ctx->last_audit_log);
        return 0;
    }
    for (i = 0; i < ctx->nonce_count; i++) {
        if (ctx->used_nonces[i] == scmd->nonce) {
            ctx->rejected_count++;
            snprintf(ctx->last_audit_log, sizeof(ctx->last_audit_log),
                     "REJECTED_REPLAY: nonce %llu already used",
                     (unsigned long long)scmd->nonce);
            if (err_out && err_len)
                snprintf(err_out, err_len, "%s", ctx->last_audit_log);
            return 0;
        }
    }

    /* 4. Compute expected HMAC-SHA256 and compare in constant time */
    snprintf(payload, sizeof(payload),
             "dev=%s|cmd=%s|pin=%s|t=%.3f|nonce=%llu",
             ctx->device_id, scmd->cmd, scmd->pin, scmd->timestamp,
             (unsigned long long)scmd->nonce);
    sec_hmac_sha256_hex(ctx->psk, payload, expected, sizeof(expected));

    if (strlen(scmd->signature) != 64 ||
        !const_time_eq(scmd->signature, expected, 64)) {
        ctx->rejected_count++;
        snprintf(ctx->last_audit_log, sizeof(ctx->last_audit_log),
                 "REJECTED_SIG: invalid HMAC-SHA256 signature");
        if (err_out && err_len)
            snprintf(err_out, err_len, "%s", ctx->last_audit_log);
        return 0;
    }

    /* 5. Record nonce in circular replay cache */
    ctx->used_nonces[ctx->nonce_head] = scmd->nonce;
    ctx->nonce_head = (ctx->nonce_head + 1) % SEC_NONCE_CACHE_SIZE;
    if (ctx->nonce_count < SEC_NONCE_CACHE_SIZE)
        ctx->nonce_count++;

    ctx->verified_count++;
    snprintf(ctx->last_audit_log, sizeof(ctx->last_audit_log),
             "VERIFIED_OK: cmd=%s nonce=%llu sig=%.8s...",
             scmd->cmd, (unsigned long long)scmd->nonce, scmd->signature);
    if (err_out && err_len)
        snprintf(err_out, err_len, "OK");
    return 1;
}
