/**
 * http_server.c — Embedded HTTP/1.1 + SSE Stream + Mock Cloud Backend + CSV Export.
 *
 * Pure C11 + POSIX/Winsock sockets. Zero external HTTP or JSON libraries.
 */
#include "http_server.h"
#include "detector.h"
#include "hal.h"
#include "security.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
   typedef int socklen_t;
#  define CLOSESOCK closesocket
#  define SOCK_ERR SOCKET_ERROR
   static int wsa_ready = 0;
#else
#  include <unistd.h>
#  include <fcntl.h>
#  include <sys/types.h>
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  define CLOSESOCK close
#  define SOCK_ERR (-1)
   typedef int SOCKET;
#  define INVALID_SOCKET (-1)
#endif

static void set_nonblock(int fd)
{
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(fd, FIONBIO, &mode);
#else
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0)
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
#endif
}

/* Escape quotes/backslashes in C strings for safe JSON embedding */
static void json_escape(const char *src, char *dst, size_t dst_len)
{
    size_t j = 0;
    if (!dst || dst_len == 0)
        return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    while (*src && j + 2 < dst_len) {
        if (*src == '"' || *src == '\\') {
            dst[j++] = '\\';
            dst[j++] = *src++;
        } else if ((unsigned char)*src < 0x20) {
            dst[j++] = ' ';
            src++;
        } else {
            dst[j++] = *src++;
        }
    }
    dst[j] = '\0';
}

int http_build_blackbox_csv(const CircularQueue *cq, char *buf, size_t buflen)
{
    TelemetryPacket snap[CQ_MAX_CAPACITY];
    size_t count, i;
    int n = 0, w;

    if (!cq || !buf || buflen < 128)
        return -1;

    w = snprintf(buf, buflen,
                 "timestamp,vehicle_id,speed_kmh,accel_x,accel_y,accel_z,"
                 "gyro_x,gyro_y,gyro_z,latitude,longitude,engine_temp_c,battery_pct,gps_valid\n");
    if (w < 0 || (size_t)w >= buflen)
        return -1;
    n += w;

    count = cq_snapshot(cq, snap, CQ_MAX_CAPACITY);
    for (i = 0; i < count; i++) {
        const TelemetryPacket *p = &snap[i];
        w = snprintf(buf + n, buflen - (size_t)n,
                     "%.4f,%d,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.1f,%.1f,%d\n",
                     p->timestamp, p->vehicle_id, p->speed,
                     p->accel_x, p->accel_y, p->accel_z,
                     p->gyro_x, p->gyro_y, p->gyro_z,
                     p->latitude, p->longitude, p->engine_temp, p->battery, p->gps_valid);
        if (w < 0 || (size_t)n + (size_t)w >= buflen)
            break;
        n += w;
    }
    return n;
}

int http_build_state_json(const EngineState *st, char *buf, size_t buflen)
{
    int n = 0, i, written;
    LinkedListNode *cur;
    const TelemetryPacket *p;
    int net_up;
    HalModemStatus mstat;
    HalLoRaStatus lstat;
    HalRelayLogEntry rlog[12];
    size_t rlog_count;
    char det_reason_esc[160], sm_status_esc[160], at_cmd_esc[96], at_resp_esc[96];
    char lora_esc[128], sec_audit_esc[160];

    if (!st || !buf || buflen < 256)
        return -1;

    p = &st->latest;
    net_up = st->publisher ? publisher_network_up(st->publisher) : 0;
    hal_modem_get_status(&mstat);
    hal_lora_get_status(&lstat);

    json_escape(st->detector ? st->detector->last_reason : "NOMINAL",
                det_reason_esc, sizeof(det_reason_esc));
    json_escape(st->sm ? st->sm->status_msg : "NORMAL",
                sm_status_esc, sizeof(sm_status_esc));
    json_escape(mstat.last_at_cmd, at_cmd_esc, sizeof(at_cmd_esc));
    json_escape(mstat.last_at_resp, at_resp_esc, sizeof(at_resp_esc));
    json_escape(lstat.last_payload, lora_esc, sizeof(lora_esc));
    json_escape(st->publisher ? st->publisher->sec.last_audit_log : "READY",
                sec_audit_esc, sizeof(sec_audit_esc));

    written = snprintf(buf + n, buflen - (size_t)n,
        "{"
        "\"t\":%.3f,"
        "\"vehicle_id\":%d,"
        "\"speed\":%.2f,"
        "\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
        "\"gyro\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
        "\"gps\":{\"lat\":%.6f,\"lon\":%.6f,\"valid\":%s,"
        "\"has_crash\":%s,\"crash_lat\":%.6f,\"crash_lon\":%.6f},"
        "\"engine_temp\":%.1f,"
        "\"battery\":%.1f,"
        "\"network_up\":%s,"
        "\"queue_count\":%lu,"
        "\"queue_capacity\":%lu,"
        "\"queue_head\":%lu,"
        "\"queue_tail\":%lu,"
        "\"queue_overwrites\":%lu,"
        "\"queue_fill\":%.3f,"
        "\"outbox_len\":%lu,"
        "\"last_alert\":\"%s\","
        "\"last_alert_time\":%.3f,"
        "\"pub_attempts\":%d,"
        "\"pub_successes\":%d,"
        "\"lora_fallbacks\":%d,"
        "\"flash_writes\":%d,"
        "\"mutations_count\":%d,"
        "\"last_hmac\":\"%.16s\","
        "\"detector\":{"
            "\"mean\":%.3f,\"var\":%.3f,\"z\":%.2f,"
            "\"amag\":%.2f,\"lin_amag\":%.2f,\"gmag\":%.2f,"
            "\"rollover_ms\":%.1f,\"brake_latched\":%s,\"overheat_latched\":%s,"
            "\"armed\":%s,\"reason\":\"%s\""
        "},"
        "\"sm\":{"
            "\"state\":\"%s\",\"status\":\"%s\","
            "\"cut_pending\":%s,\"overrides\":%d"
        "},"
        "\"relays\":{"
            "\"engine\":%d,\"siren\":%d,\"lights\":%d,\"lock\":%d"
        "},"
        "\"modem\":{"
            "\"attached\":%s,\"rssi\":%d,\"gnss_fix\":%s,"
            "\"at_cmd\":\"%s\",\"at_resp\":\"%s\""
        "},"
        "\"lora\":{"
            "\"sent\":%d,\"rssi\":%d,\"last_payload\":\"%s\""
        "},"
        "\"security\":{"
            "\"device_id\":\"%s\",\"verified\":%d,\"rejected\":%d,\"audit\":\"%s\""
        "},",
        st->sim_time,
        p->vehicle_id,
        p->speed,
        p->accel_x, p->accel_y, p->accel_z,
        p->gyro_x, p->gyro_y, p->gyro_z,
        p->latitude, p->longitude, p->gps_valid ? "true" : "false",
        st->has_crash_loc ? "true" : "false", st->crash_lat, st->crash_lon,
        p->engine_temp,
        p->battery,
        net_up ? "true" : "false",
        (unsigned long)(st->blackbox ? cq_count(st->blackbox) : 0),
        (unsigned long)(st->blackbox ? st->blackbox->capacity : 0),
        (unsigned long)(st->blackbox ? st->blackbox->head : 0),
        (unsigned long)(st->blackbox ? st->blackbox->tail : 0),
        (unsigned long)(st->blackbox ? st->blackbox->overwrites : 0),
        (st->blackbox && st->blackbox->capacity)
            ? (double)cq_count(st->blackbox) / (double)st->blackbox->capacity
            : 0.0,
        (unsigned long)(st->publisher ? ll_count(&st->publisher->outbox) : 0),
        alert_type_name(st->last_alert_type),
        st->last_alert_time,
        st->publisher ? st->publisher->publish_attempts : 0,
        st->publisher ? st->publisher->publish_successes : 0,
        st->publisher ? st->publisher->lora_fallbacks : 0,
        st->publisher ? st->publisher->flash_writes : 0,
        st->publisher ? st->publisher->mutations_count : 0,
        (st->publisher && st->publisher->last_alert_sig[0]) ? st->publisher->last_alert_sig : "none",
        st->detector ? st->detector->mean : 0.0,
        st->detector ? st->detector->m2 : 0.0,
        st->detector ? st->detector->last_z : 0.0,
        st->detector ? st->detector->last_accel_mag : 0.0,
        st->detector ? st->detector->last_linear_accel_mag : 0.0,
        st->detector ? st->detector->last_gyro_mag : 0.0,
        st->detector ? st->detector->rollover_accum_ms : 0.0,
        (st->detector && st->detector->brake_active) ? "true" : "false",
        (st->detector && st->detector->overheat_active) ? "true" : "false",
        (st->detector && st->detector->armed) ? "true" : "false",
        det_reason_esc,
        st->sm ? sm_state_name(st->sm->state) : "NORMAL",
        sm_status_esc,
        (st->sm && st->sm->engine_cut_pending) ? "true" : "false",
        st->sm ? st->sm->override_count : 0,
        hal_relay_get(HAL_RELAY_ENGINE),
        hal_relay_get(HAL_RELAY_SIREN),
        hal_relay_get(HAL_RELAY_LIGHTS),
        hal_relay_get(HAL_RELAY_LOCK),
        mstat.attached ? "true" : "false",
        mstat.rssi_dbm,
        mstat.gnss_fix ? "true" : "false",
        at_cmd_esc,
        at_resp_esc,
        lstat.packets_sent,
        lstat.rssi_dbm,
        lora_esc,
        CFG_DEVICE_ID,
        st->publisher ? st->publisher->sec.verified_count : 0,
        st->publisher ? st->publisher->sec.rejected_count : 0,
        sec_audit_esc
    );
    if (written < 0 || (size_t)written >= buflen - (size_t)n)
        return -1;
    n += written;

    /* Relay actuation log */
    written = snprintf(buf + n, buflen - (size_t)n, "\"relay_log\":[");
    if (written < 0) return -1;
    n += written;
    rlog_count = hal_relay_get_log(rlog, 12);
    for (i = 0; i < (int)rlog_count; i++) {
        char r_esc[64];
        json_escape(rlog[i].reason, r_esc, sizeof(r_esc));
        written = snprintf(buf + n, buflen - (size_t)n,
            "%s{\"t\":%.2f,\"relay\":\"%s\",\"state\":%d,\"reason\":\"%s\"}",
            i ? "," : "", rlog[i].timestamp, hal_relay_name(rlog[i].relay),
            rlog[i].state, r_esc);
        if (written < 0 || (size_t)n + (size_t)written >= buflen) return -1;
        n += written;
    }

    /* alerts array from outbox linked list */
    written = snprintf(buf + n, buflen - (size_t)n, "],\"alerts\":[");
    if (written < 0) return -1;
    n += written;

    if (st->publisher) {
        int first = 1;
        for (cur = st->publisher->outbox.head; cur; cur = cur->next) {
            const Alert *a = &cur->data;
            double retry_in = a->next_retry_time - st->sim_time;
            if (retry_in < 0.0) retry_in = 0.0;
            written = snprintf(buf + n, buflen - (size_t)n,
                "%s{\"id\":%d,\"type\":\"%s\",\"severity\":\"%s\","
                "\"t\":%.3f,\"lat\":%.6f,\"lon\":%.6f,\"retries\":%d,\"retry_in\":%.1f}",
                first ? "" : ",",
                a->id, alert_type_name(a->type), severity_name(a->severity),
                a->timestamp, a->lat, a->lon, a->retry_count, retry_in);
            if (written < 0 || (size_t)n + (size_t)written >= buflen)
                return -1;
            n += written;
            first = 0;
        }
    }

    /* Recent blackbox frames for interactive timeline scrubber (up to last 32 frames) */
    written = snprintf(buf + n, buflen - (size_t)n, "],\"snapshot\":[");
    if (written < 0) return -1;
    n += written;
    if (st->blackbox) {
        TelemetryPacket snap[CQ_MAX_CAPACITY];
        size_t total = cq_snapshot(st->blackbox, snap, CQ_MAX_CAPACITY);
        size_t start_idx = (total > 32) ? (total - 32) : 0;
        size_t k;
        for (k = start_idx; k < total; k++) {
            const TelemetryPacket *sp = &snap[k];
            written = snprintf(buf + n, buflen - (size_t)n,
                "%s{\"t\":%.2f,\"spd\":%.1f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
                "\"gx\":%.1f,\"gy\":%.1f,\"temp\":%.1f,\"lat\":%.5f,\"lon\":%.5f}",
                (k == start_idx) ? "" : ",",
                sp->timestamp, sp->speed, sp->accel_x, sp->accel_y, sp->accel_z,
                sp->gyro_x, sp->gyro_y, sp->engine_temp, sp->latitude, sp->longitude);
            if (written < 0 || (size_t)n + (size_t)written >= buflen)
                return -1;
            n += written;
        }
    }

    written = snprintf(buf + n, buflen - (size_t)n, "],\"plot\":{\"ax\":[");
    if (written < 0) return -1;
    n += written;

    /* Plot ring: chronological order */
    {
        int start = (st->plot_count < 64) ? 0 : (st->plot_idx % 64);
        int count = st->plot_count < 64 ? st->plot_count : 64;
        for (i = 0; i < count; i++) {
            int idx = (start + i) % 64;
            written = snprintf(buf + n, buflen - (size_t)n, "%s%.3f",
                               i ? "," : "", st->plot_ax[idx]);
            if (written < 0) return -1;
            n += written;
        }
        written = snprintf(buf + n, buflen - (size_t)n, "],\"ay\":[");
        if (written < 0) return -1;
        n += written;
        for (i = 0; i < count; i++) {
            int idx = (start + i) % 64;
            written = snprintf(buf + n, buflen - (size_t)n, "%s%.3f",
                               i ? "," : "", st->plot_ay[idx]);
            if (written < 0) return -1;
            n += written;
        }
        written = snprintf(buf + n, buflen - (size_t)n, "],\"az\":[");
        if (written < 0) return -1;
        n += written;
        for (i = 0; i < count; i++) {
            int idx = (start + i) % 64;
            written = snprintf(buf + n, buflen - (size_t)n, "%s%.3f",
                               i ? "," : "", st->plot_az[idx]);
            if (written < 0) return -1;
            n += written;
        }
    }

    written = snprintf(buf + n, buflen - (size_t)n, "]}}");
    if (written < 0) return -1;
    n += written;
    return n;
}

static int read_file_all(const char *path, char **out, size_t *out_len)
{
    FILE *fp;
    long sz;
    char *buf;
    fp = fopen(path, "rb");
    if (!fp)
        return -1;
    if (fseek(fp, 0, SEEK_END) != 0) { fclose(fp); return -1; }
    sz = ftell(fp);
    if (sz < 0) { fclose(fp); return -1; }
    rewind(fp);
    buf = (char *)malloc((size_t)sz + 1);
    if (!buf) { fclose(fp); return -1; }
    if (fread(buf, 1, (size_t)sz, fp) != (size_t)sz) {
        free(buf);
        fclose(fp);
        return -1;
    }
    buf[sz] = '\0';
    fclose(fp);
    *out = buf;
    *out_len = (size_t)sz;
    return 0;
}

static int send_all(int fd, const char *data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
#ifdef _WIN32
        int n = send(fd, data + sent, (int)(len - sent), 0);
#else
        ssize_t n = send(fd, data + sent, len - sent, 0);
#endif
        if (n <= 0)
            return -1;
        sent += (size_t)n;
    }
    return 0;
}

static void respond_ex(int fd, int code, const char *ctype,
                       const char *extra_hdr, const char *body, size_t body_len)
{
    char hdr[768];
    int hlen = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %lu\r\n"
        "Connection: close\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-store\r\n"
        "%s"
        "\r\n",
        code, code == 200 ? "OK" : (code == 403 ? "Forbidden" : (code == 404 ? "Not Found" : "Error")),
        ctype, (unsigned long)body_len, extra_hdr ? extra_hdr : "");
    if (hlen > 0)
        send_all(fd, hdr, (size_t)hlen);
    if (body && body_len)
        send_all(fd, body, body_len);
}

static void respond(int fd, int code, const char *ctype,
                    const char *body, size_t body_len)
{
    respond_ex(fd, code, ctype, NULL, body, body_len);
}

/* Simple query parameter extractor: finds key=value in query string */
static void get_query_param(const char *query, const char *key, char *out, size_t out_len)
{
    const char *p;
    size_t klen;
    if (!out || out_len == 0) return;
    out[0] = '\0';
    if (!query || !key) return;
    klen = strlen(key);
    p = query;
    while ((p = strstr(p, key)) != NULL) {
        if ((p == query || p[-1] == '?' || p[-1] == '&') && p[klen] == '=') {
            const char *val = p + klen + 1;
            size_t i = 0;
            while (val[i] && val[i] != '&' && val[i] != ' ' && i + 1 < out_len) {
                out[i] = val[i];
                i++;
            }
            out[i] = '\0';
            return;
        }
        p += klen;
    }
}

static void handle_control_request(HttpServer *srv, int client, const char *path_and_query)
{
    EngineState *st = srv->state;
    char action[64], sc_name[64], pin[32];
    char resp_json[256];

    get_query_param(path_and_query, "action", action, sizeof(action));
    get_query_param(path_and_query, "scenario", sc_name, sizeof(sc_name));
    get_query_param(path_and_query, "pin", pin, sizeof(pin));

    if (!strcmp(action, "inject") && st->sim) {
        Scenario sc = scenario_from_name(sc_name);
        if (sc == SCENARIO_THEFT && st->detector)
            detector_set_armed(st->detector, 1);
        simulator_inject(st->sim, sc);
        snprintf(resp_json, sizeof(resp_json),
                 "{\"ok\":true,\"msg\":\"Injected scenario %s\"}", scenario_name(sc));
        respond(client, 200, "application/json", resp_json, strlen(resp_json));
    } else if (!strcmp(action, "network") && st->publisher) {
        publisher_toggle_network(st->publisher);
        snprintf(resp_json, sizeof(resp_json),
                 "{\"ok\":true,\"network_up\":%s}",
                 publisher_network_up(st->publisher) ? "true" : "false");
        respond(client, 200, "application/json", resp_json, strlen(resp_json));
    } else if (!strcmp(action, "arm") && st->detector) {
        int new_arm = !st->detector->armed;
        detector_set_armed(st->detector, new_arm);
        snprintf(resp_json, sizeof(resp_json),
                 "{\"ok\":true,\"armed\":%s}", new_arm ? "true" : "false");
        respond(client, 200, "application/json", resp_json, strlen(resp_json));
    } else if (!strcmp(action, "autodemo") && st->sim) {
        int next_demo = !st->sim->auto_demo;
        simulator_set_autodemo(st->sim, next_demo);
        snprintf(resp_json, sizeof(resp_json),
                 "{\"ok\":true,\"auto_demo\":%s}", next_demo ? "true" : "false");
        respond(client, 200, "application/json", resp_json, strlen(resp_json));
    } else if (!strcmp(action, "override") && st->sm) {
        int ok = sm_override_pin(st->sm, pin, st->sim_time);
        if (ok && st->detector)
            detector_set_armed(st->detector, 0);
        snprintf(resp_json, sizeof(resp_json),
                 "{\"ok\":%s,\"msg\":\"%s\"}",
                 ok ? "true" : "false", st->sm->status_msg);
        respond(client, ok ? 200 : 403, "application/json", resp_json, strlen(resp_json));
    } else {
        const char *err = "{\"ok\":false,\"msg\":\"unknown control action\"}";
        respond(client, 400, "application/json", err, strlen(err));
    }
}

static void handle_cloud_command(HttpServer *srv, int client, const char *path_and_query)
{
    EngineState *st = srv->state;
    char cmd_name[32], pin[16], mode[32], err_msg[128], resp_json[384];
    SignedCommand scmd;
    int verified;

    if (!st || !st->publisher || !st->sm) {
        const char *e = "{\"ok\":false,\"msg\":\"engine not ready\"}";
        respond(client, 500, "application/json", e, strlen(e));
        return;
    }

    get_query_param(path_and_query, "cmd", cmd_name, sizeof(cmd_name));
    get_query_param(path_and_query, "pin", pin, sizeof(pin));
    get_query_param(path_and_query, "mode", mode, sizeof(mode));

    memset(&scmd, 0, sizeof(scmd));
    snprintf(scmd.cmd, sizeof(scmd.cmd), "%s", cmd_name[0] ? cmd_name : "DISARM");
    snprintf(scmd.pin, sizeof(scmd.pin), "%s", pin[0] ? pin : CFG_DEFAULT_OVERRIDE_PIN);
    scmd.timestamp = st->sim_time;

    if (!strcmp(mode, "replay")) {
        /* Reuse previous nonce to demonstrate anti-replay rejection */
        scmd.nonce = srv->cmd_nonce_counter ? srv->cmd_nonce_counter : 1000ULL;
        sec_sign_command(&st->publisher->sec, &scmd);
    } else if (!strcmp(mode, "stale")) {
        srv->cmd_nonce_counter++;
        scmd.nonce = srv->cmd_nonce_counter;
        scmd.timestamp = st->sim_time - 120.0; /* 120s old (> 30s limit) */
        sec_sign_command(&st->publisher->sec, &scmd);
    } else if (!strcmp(mode, "tamper")) {
        srv->cmd_nonce_counter++;
        scmd.nonce = srv->cmd_nonce_counter;
        sec_sign_command(&st->publisher->sec, &scmd);
        scmd.signature[0] = (scmd.signature[0] == 'a') ? 'b' : 'a';
    } else {
        /* Valid fresh signed command from mock cloud */
        srv->cmd_nonce_counter++;
        scmd.nonce = srv->cmd_nonce_counter;
        sec_sign_command(&st->publisher->sec, &scmd);
    }

    verified = sec_verify_command(&st->publisher->sec, &scmd, st->sim_time,
                                  err_msg, sizeof(err_msg));
    if (verified) {
        if (!strcmp(scmd.cmd, "DISARM")) {
            sm_override_pin(st->sm, scmd.pin, st->sim_time);
            if (st->detector) detector_set_armed(st->detector, 0);
        } else if (!strcmp(scmd.cmd, "LOCKDOWN")) {
            sm_enter_lockdown(st->sm, st->sim_time, st->latest.speed, "cloud_signed_lockdown");
        } else if (!strcmp(scmd.cmd, "ARM")) {
            if (st->detector) detector_set_armed(st->detector, 1);
        }
    }

    {
        char audit_esc[160];
        json_escape(st->publisher->sec.last_audit_log, audit_esc, sizeof(audit_esc));
        snprintf(resp_json, sizeof(resp_json),
                 "{\"ok\":%s,\"cmd\":\"%s\",\"nonce\":%llu,\"sig\":\"%.16s...\","
                 "\"audit\":\"%s\"}",
                 verified ? "true" : "false",
                 scmd.cmd, (unsigned long long)scmd.nonce,
                 scmd.signature, audit_esc);
        respond(client, verified ? 200 : 403, "application/json",
                resp_json, strlen(resp_json));
    }
}

static void add_sse_client(HttpServer *srv, int client)
{
    int i;
    const char *sse_hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n";

    set_nonblock(client);
    if (send_all(client, sse_hdr, strlen(sse_hdr)) != 0) {
        CLOSESOCK(client);
        return;
    }

    for (i = 0; i < HTTP_MAX_SSE_CLIENTS; i++) {
        if (srv->sse_fds[i] == (int)INVALID_SOCKET) {
            srv->sse_fds[i] = client;
            return;
        }
    }
    /* Evict slot 0 if full */
    CLOSESOCK(srv->sse_fds[0]);
    srv->sse_fds[0] = client;
}

void http_server_broadcast_sse(HttpServer *srv)
{
    char json[HTTP_JSON_BUF];
    char frame[HTTP_JSON_BUF + 32];
    int jlen, flen, i;
    int any_client = 0;

    if (!srv || !srv->state)
        return;

    for (i = 0; i < HTTP_MAX_SSE_CLIENTS; i++) {
        if (srv->sse_fds[i] != (int)INVALID_SOCKET) {
            any_client = 1;
            break;
        }
    }
    if (!any_client)
        return;

    jlen = http_build_state_json(srv->state, json, sizeof(json));
    if (jlen < 0)
        return;

    flen = snprintf(frame, sizeof(frame), "data: %s\n\n", json);
    if (flen <= 0)
        return;

    for (i = 0; i < HTTP_MAX_SSE_CLIENTS; i++) {
        int fd = srv->sse_fds[i];
        if (fd != (int)INVALID_SOCKET) {
            if (send_all(fd, frame, (size_t)flen) != 0) {
                CLOSESOCK(fd);
                srv->sse_fds[i] = (int)INVALID_SOCKET;
            }
        }
    }
}

static void handle_client(HttpServer *srv, int client)
{
    char req[2048];
    int n;
    char method[16], full_path[256], clean_path[256];
    char *qmark;
    char buf[HTTP_JSON_BUF];
    int blen;
    char *file_buf = NULL;
    size_t file_len = 0;

#ifdef _WIN32
    n = recv(client, req, sizeof(req) - 1, 0);
#else
    n = (int)recv(client, req, sizeof(req) - 1, 0);
#endif
    if (n <= 0) {
        CLOSESOCK(client);
        return;
    }
    req[n] = '\0';

    method[0] = full_path[0] = '\0';
    sscanf(req, "%15s %255s", method, full_path);

    snprintf(clean_path, sizeof(clean_path), "%s", full_path);
    qmark = strchr(clean_path, '?');
    if (qmark) *qmark = '\0';

    if (strcmp(method, "OPTIONS") == 0) {
        respond(client, 200, "text/plain", "", 0);
        CLOSESOCK(client);
        return;
    }

    if (strcmp(clean_path, "/api/stream") == 0) {
        add_sse_client(srv, client);
        http_server_broadcast_sse(srv);
        return; /* keep socket open for SSE stream */
    }

    if (strcmp(clean_path, "/api/control") == 0) {
        handle_control_request(srv, client, full_path);
        CLOSESOCK(client);
        return;
    }

    if (strcmp(clean_path, "/api/cloud/command") == 0) {
        handle_cloud_command(srv, client, full_path);
        CLOSESOCK(client);
        return;
    }

    if (strcmp(clean_path, "/api/state") == 0) {
        blen = http_build_state_json(srv->state, buf, sizeof(buf));
        if (blen < 0) {
            const char *msg = "{\"error\":\"overflow\"}";
            respond(client, 500, "application/json", msg, strlen(msg));
        } else {
            respond(client, 200, "application/json", buf, (size_t)blen);
        }
    } else if (strcmp(clean_path, "/api/blackbox.csv") == 0) {
        blen = http_build_blackbox_csv(srv->state ? srv->state->blackbox : NULL,
                                       buf, sizeof(buf));
        if (blen < 0) {
            const char *msg = "error generating CSV";
            respond(client, 500, "text/plain", msg, strlen(msg));
        } else {
            respond_ex(client, 200, "text/csv; charset=utf-8",
                       "Content-Disposition: attachment; filename=\"blackbox_forensics.csv\"\r\n",
                       buf, (size_t)blen);
        }
    } else if (strcmp(clean_path, "/manifest.json") == 0) {
        if (read_file_all("dashboard/manifest.json", &file_buf, &file_len) == 0 ||
            read_file_all("../dashboard/manifest.json", &file_buf, &file_len) == 0) {
            respond(client, 200, "application/manifest+json", file_buf, file_len);
            free(file_buf);
        } else {
            respond(client, 404, "text/plain", "Not Found", 9);
        }
    } else if (strcmp(clean_path, "/sw.js") == 0) {
        if (read_file_all("dashboard/sw.js", &file_buf, &file_len) == 0 ||
            read_file_all("../dashboard/sw.js", &file_buf, &file_len) == 0) {
            respond(client, 200, "application/javascript", file_buf, file_len);
            free(file_buf);
        } else {
            respond(client, 404, "text/plain", "Not Found", 9);
        }
    } else if (strcmp(clean_path, "/") == 0 ||
               strcmp(clean_path, "/dashboard.html") == 0 ||
               strcmp(clean_path, "/index.html") == 0) {
        if (read_file_all("dashboard/index.html", &file_buf, &file_len) == 0 ||
            read_file_all("dashboard/dashboard.html", &file_buf, &file_len) == 0 ||
            read_file_all("../dashboard/index.html", &file_buf, &file_len) == 0) {
            respond(client, 200, "text/html; charset=utf-8", file_buf, file_len);
            free(file_buf);
        } else {
            const char *msg = "dashboard/index.html not found — run from project root";
            respond(client, 404, "text/plain", msg, strlen(msg));
        }
    } else {
        const char *msg = "Not Found";
        respond(client, 404, "text/plain", msg, strlen(msg));
    }
    CLOSESOCK(client);
}

int http_server_init(HttpServer *srv, int port, EngineState *state)
{
    struct sockaddr_in addr;
    int yes = 1;
    int i;

    if (!srv)
        return -1;
    memset(srv, 0, sizeof(*srv));
    srv->port = port;
    srv->state = state;
    srv->listen_fd = (int)INVALID_SOCKET;
    srv->cmd_nonce_counter = 1000UL;
    for (i = 0; i < HTTP_MAX_SSE_CLIENTS; i++)
        srv->sse_fds[i] = (int)INVALID_SOCKET;

#ifdef _WIN32
    if (!wsa_ready) {
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
            return -1;
        wsa_ready = 1;
    }
#endif

    srv->listen_fd = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (srv->listen_fd == (int)INVALID_SOCKET)
        return -1;

    setsockopt(srv->listen_fd, SOL_SOCKET, SO_REUSEADDR,
               (const char *)&yes, sizeof(yes));

    {
        int try_p, bound = 0;
        for (try_p = port; try_p <= port + 10; try_p++) {
            memset(&addr, 0, sizeof(addr));
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
            addr.sin_port = htons((unsigned short)try_p);
            if (bind(srv->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != SOCK_ERR) {
                srv->port = try_p;
                bound = 1;
                break;
            }
        }
        if (!bound) {
            fprintf(stderr, "[HTTP] bind failed: %s\n", strerror(errno));
            CLOSESOCK(srv->listen_fd);
            srv->listen_fd = (int)INVALID_SOCKET;
            return -1;
        }
    }
    if (listen(srv->listen_fd, 16) == SOCK_ERR) {
        CLOSESOCK(srv->listen_fd);
        srv->listen_fd = (int)INVALID_SOCKET;
        return -1;
    }
    set_nonblock(srv->listen_fd);
    printf("[HTTP] Live SSE Dashboard & Mock Cloud on http://127.0.0.1:%d/\n", srv->port);
    return 0;
}

void http_server_poll(HttpServer *srv)
{
    struct sockaddr_in cli;
    socklen_t clen = sizeof(cli);
    int client;
    int budget = 4;

    if (!srv || srv->listen_fd == (int)INVALID_SOCKET)
        return;

    while (budget-- > 0) {
        client = (int)accept(srv->listen_fd, (struct sockaddr *)&cli, &clen);
        if (client == (int)INVALID_SOCKET)
            break;
        handle_client(srv, client);
    }
}

void http_server_shutdown(HttpServer *srv)
{
    int i;
    if (!srv)
        return;
    for (i = 0; i < HTTP_MAX_SSE_CLIENTS; i++) {
        if (srv->sse_fds[i] != (int)INVALID_SOCKET) {
            CLOSESOCK(srv->sse_fds[i]);
            srv->sse_fds[i] = (int)INVALID_SOCKET;
        }
    }
    if (srv->listen_fd != (int)INVALID_SOCKET) {
        CLOSESOCK(srv->listen_fd);
        srv->listen_fd = (int)INVALID_SOCKET;
    }
#ifdef _WIN32
    if (wsa_ready) {
        WSACleanup();
        wsa_ready = 0;
    }
#endif
}
