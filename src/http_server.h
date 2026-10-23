/**
 * http_server.h — Embedded HTTP/1.1 + Server-Sent Events (SSE) + Mock Cloud Server.
 *
 * Endpoints:
 *   GET  / or /index.html or /dashboard.html -> Live Dashboard PWA
 *   GET  /manifest.json                      -> PWA Web App Manifest
 *   GET  /sw.js                              -> PWA Service Worker (offline shell)
 *   GET  /api/stream                         -> Server-Sent Events (SSE) real-time push
 *   GET  /api/state                          -> Full JSON state snapshot
 *   GET  /api/blackbox.csv                   -> Forensic black-box CSV export
 *   POST /api/control                        -> Scenario inject, network toggle, PIN override, arm
 *   POST /api/cloud/command                  -> Signed remote command endpoint (HMAC-SHA256 + nonce)
 */
#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include "circular_queue.h"
#include "detector.h"
#include "publisher.h"
#include "simulator.h"
#include "state_machine.h"
#include "structs.h"

#define HTTP_PORT           8080
#define HTTP_JSON_BUF       65536
#define HTTP_MAX_SSE_CLIENTS 8

typedef struct {
    TelemetryPacket latest;
    CircularQueue  *blackbox;
    Detector       *detector;
    Publisher      *publisher;
    Simulator      *sim;
    StateMachine   *sm;
    int             running;
    double          sim_time;
    AlertType       last_alert_type;
    double          last_alert_time;
    double          crash_lat;
    double          crash_lon;
    int             has_crash_loc;
    /* recent accel samples for the live plot (ring of doubles) */
    double          plot_ax[64];
    double          plot_ay[64];
    double          plot_az[64];
    int             plot_idx;
    int             plot_count;
} EngineState;

typedef struct {
    int          listen_fd;
    int          port;
    int          sse_fds[HTTP_MAX_SSE_CLIENTS];
    unsigned long cmd_nonce_counter;
    EngineState *state;
} HttpServer;

int  http_server_init(HttpServer *srv, int port, EngineState *state);
void http_server_poll(HttpServer *srv);          /* non-blocking accept + handle HTTP */
void http_server_broadcast_sse(HttpServer *srv); /* push live state to SSE clients    */
void http_server_shutdown(HttpServer *srv);

/** Build /api/state JSON into buf. Returns length, or -1 on error. */
int  http_build_state_json(const EngineState *st, char *buf, size_t buflen);

/** Build CSV export of the black-box circular queue into buf. */
int  http_build_blackbox_csv(const CircularQueue *cq, char *buf, size_t buflen);

#endif /* HTTP_SERVER_H */
