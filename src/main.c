/**
 * main.c — Telemetry & Anomaly Reconstruction Engine entry point.
 *
 * Dual-rate architecture (Phase 1):
 *   - High-rate sensing loop (250 Hz): hal_imu_read_sample -> detector_process O(1)
 *   - Slow telemetry loop (5 Hz): temp, speed, voltage, GPS, black-box enqueue,
 *     publisher outbox retry, and HTTP dashboard polling.
 *
 * Keys (while running):
 *   c = collision, b = hard brake, r = rollover, o = overheat,
 *   p = pothole noise, t = theft, g = gps loss, f = sensor fault,
 *   n = toggle network, q = quit
 */
#include "config.h"
#include "circular_queue.h"
#include "detector.h"
#include "hal.h"
#include "http_server.h"
#include "publisher.h"
#include "simulator.h"
#include "structs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>

#ifdef _WIN32
#  include <windows.h>
#  include <conio.h>
#else
#  include <unistd.h>
#  include <termios.h>
#  include <fcntl.h>
#  include <sys/select.h>
#endif

static volatile int g_running = 1;

static void on_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ---- non-blocking keyboard ---------------------------------------------- */

#ifndef _WIN32
static struct termios g_oterm;
static int g_term_set = 0;

static void term_raw_on(void)
{
    struct termios t;
    if (tcgetattr(STDIN_FILENO, &g_oterm) == 0) {
        t = g_oterm;
        t.c_lflag &= (tcflag_t)~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
        g_term_set = 1;
    }
}
static void term_raw_off(void)
{
    if (g_term_set)
        tcsetattr(STDIN_FILENO, TCSANOW, &g_oterm);
}
#endif

static int kb_poll(void)
{
#ifdef _WIN32
    if (_kbhit())
        return _getch();
    return -1;
#else
    struct timeval tv = {0, 0};
    fd_set fds;
    unsigned char c;
    if (!g_term_set)
        return -1;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
        if (read(STDIN_FILENO, &c, 1) == 1)
            return (int)c;
    }
    return -1;
#endif
}

/* ---- pre-crash timeline reconstruction ---------------------------------- */

static void reconstruct_timeline(const CircularQueue *cq,
                                 const Alert *alert,
                                 const TelemetryPacket *trigger)
{
    TelemetryPacket window[CQ_DEFAULT_CAPACITY];
    size_t n, i;
    char line[256];
    int len;

    n = cq_snapshot(cq, window, CQ_DEFAULT_CAPACITY);

    printf("\n======== ANOMALY TIMELINE ========\n");
    printf("Type: %s  Severity: %s\n",
           alert_type_name(alert->type), severity_name(alert->severity));
    printf("Exact localization: lat=%.6f lon=%.6f  t=%.3f\n",
           trigger->latitude, trigger->longitude, trigger->timestamp);
    printf("Pre-crash black-box window (%lu packets):\n", (unsigned long)n);
    for (i = 0; i < n; i++) {
        const TelemetryPacket *p = &window[i];
        printf("  t=%6.2f  spd=%5.1f  ax=%6.2f ay=%6.2f az=%6.2f  "
               "temp=%.1f  gps=(%.5f,%.5f)%s\n",
               p->timestamp, p->speed, p->accel_x, p->accel_y, p->accel_z,
               p->engine_temp, p->latitude, p->longitude,
               (fabs(p->timestamp - trigger->timestamp) < 1e-6) ? "  <-- TRIGGER" : "");
    }
    printf("==================================\n\n");

    len = snprintf(line, sizeof(line),
                   "EVENT %s sev=%s t=%.3f lat=%.6f lon=%.6f packets=%lu\n",
                   alert_type_name(alert->type), severity_name(alert->severity),
                   trigger->timestamp, trigger->latitude, trigger->longitude,
                   (unsigned long)n);
    if (len > 0)
        hal_storage_append_file(CFG_TIMELINE_PATH, line, (size_t)len);

    for (i = 0; i < n; i++) {
        const TelemetryPacket *p = &window[i];
        len = snprintf(line, sizeof(line),
                       "  %.3f,%d,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.1f,%.1f\n",
                       p->timestamp, p->vehicle_id, p->speed,
                       p->accel_x, p->accel_y, p->accel_z,
                       p->gyro_x, p->gyro_y, p->gyro_z,
                       p->latitude, p->longitude, p->engine_temp, p->battery);
        if (len > 0)
            hal_storage_append_file(CFG_TIMELINE_PATH, line, (size_t)len);
    }
    hal_storage_append_file(CFG_TIMELINE_PATH, "END\n", 4);
}

static void plot_push(EngineState *st, const TelemetryPacket *p)
{
    st->plot_ax[st->plot_idx] = p->accel_x;
    st->plot_ay[st->plot_idx] = p->accel_y;
    st->plot_az[st->plot_idx] = p->accel_z;
    st->plot_idx = (st->plot_idx + 1) % 64;
    if (st->plot_count < 64)
        st->plot_count++;
}

static void print_help(const char *argv0)
{
    printf("Usage: %s [options]\n"
           "  --scenario NAME   collision|hard_brake|rollover|overheat|pothole|cruise|theft|gps_loss|sensor_fault\n"
           "  --at SECONDS      when to fire the scenario (default 5)\n"
           "  --seed N          deterministic PRNG seed\n"
           "  --auto-demo       run scripted multi-scenario demonstration\n"
           "  --port N          HTTP port (default %d)\n"
           "  --disconnect      start with network forced down\n"
           "  --vehicle ID      vehicle id (default 42)\n"
           "  --no-http         disable dashboard server\n"
           "  --duration SEC    auto-quit after SEC simulation seconds\n"
           "  --help            this message\n"
           "\nKeys while running: c/b/r/o/p/t/g/f = scenarios, n = network, q = quit\n",
           argv0, HTTP_PORT);
}

int main(int argc, char **argv)
{
    CircularQueue blackbox;
    Detector detector;
    Publisher publisher;
    Simulator sim;
    StateMachine sm;
    HttpServer http;
    EngineState state;
    Scenario cli_scenario = SCENARIO_NONE;
    double cli_at = 5.0;
    int port = HTTP_PORT;
    int vehicle_id = 42;
    int use_http = 1;
    int start_down = 0;
    int auto_demo = 0;
    unsigned seed = 0;
    int has_seed = 0;
    double max_duration = -1.0;
    int i;
    int http_ok = 0;

    setvbuf(stdout, NULL, _IONBF, 0);

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            print_help(argv[0]);
            return 0;
        } else if (!strcmp(argv[i], "--scenario") && i + 1 < argc) {
            cli_scenario = scenario_from_name(argv[++i]);
            if (cli_scenario == SCENARIO_NONE) {
                fprintf(stderr, "Unknown scenario '%s'\n", argv[i]);
                return 1;
            }
        } else if (!strcmp(argv[i], "--at") && i + 1 < argc) {
            cli_at = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--seed") && i + 1 < argc) {
            seed = (unsigned)strtoul(argv[++i], NULL, 10);
            has_seed = 1;
        } else if (!strcmp(argv[i], "--auto-demo")) {
            auto_demo = 1;
        } else if (!strcmp(argv[i], "--port") && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--vehicle") && i + 1 < argc) {
            vehicle_id = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--disconnect")) {
            start_down = 1;
        } else if (!strcmp(argv[i], "--no-http")) {
            use_http = 0;
        } else if (!strcmp(argv[i], "--duration") && i + 1 < argc) {
            max_duration = atof(argv[++i]);
        } else {
            fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            print_help(argv[0]);
            return 1;
        }
    }

    signal(SIGINT, on_signal);
#ifdef SIGTERM
    signal(SIGTERM, on_signal);
#endif
#ifdef SIGPIPE
    signal(SIGPIPE, SIG_IGN);
#endif

    /* Initialize Hardware Abstraction Layer (sim backend on desktop) */
    hal_storage_init();
    hal_imu_init();
    hal_relay_init_failsafe();
    hal_modem_init();
    hal_lora_init();
    hal_watchdog_init(CFG_SM_WATCHDOG_TIMEOUT_S);

    if (cq_init(&blackbox, CQ_DEFAULT_CAPACITY) != 0) {
        fprintf(stderr, "Failed to init circular queue\n");
        return 1;
    }
    detector_init(&detector);
    detector_set_sample_rate(&detector, CFG_IMU_SAMPLE_HZ);
    publisher_init(&publisher);
    simulator_init(&sim, vehicle_id);
    sm_init(&sm);
    if (has_seed)
        simulator_set_seed(&sim, seed);
    if (auto_demo)
        simulator_set_autodemo(&sim, 1);

    /* Continue alert ids past anything reloaded from the durable outbox. */
    {
        LinkedListNode *cur;
        int max_id = 0;
        for (cur = publisher.outbox.head; cur; cur = cur->next) {
            if (cur->data.id > max_id)
                max_id = cur->data.id;
        }
        if (max_id > 0)
            detector_set_next_id(&detector, max_id + 1);
    }

    if (start_down) {
        publisher_set_network(&publisher, 0);
        hal_modem_set_link_up(0);
    }
    if (cli_scenario != SCENARIO_NONE)
        simulator_schedule(&sim, cli_scenario, cli_at);

    memset(&state, 0, sizeof(state));
    state.blackbox  = &blackbox;
    state.detector  = &detector;
    state.publisher = &publisher;
    state.sim       = &sim;
    state.sm        = &sm;
    state.running   = 1;

    if (use_http) {
        if (http_server_init(&http, port, &state) != 0) {
            fprintf(stderr, "HTTP server failed to bind port %d (continuing without UI)\n",
                    port);
            http_ok = 0;
        } else {
            http_ok = 1;
        }
    }

#ifndef _WIN32
    term_raw_on();
#endif

    {
        const HalImuCalibration *cal = hal_imu_get_calibration();
        printf("Telemetry & Anomaly Reconstruction Engine (IMU=%dHz, Telemetry=%dHz)\n",
               CFG_IMU_SAMPLE_HZ, CFG_TELEMETRY_HZ);
        if (cal && cal->calibrated) {
            printf("[CAL] Boot IMU calibration OK: accel_bias=(%.3f, %.3f, %.3f) "
                   "gyro_bias=(%.3f, %.3f, %.3f) gravity_removed=%s\n",
                   cal->accel_bias[0], cal->accel_bias[1], cal->accel_bias[2],
                   cal->gyro_bias[0], cal->gyro_bias[1], cal->gyro_bias[2],
                   cal->remove_gravity ? "YES" : "NO");
        }
        printf("Black-box capacity=%lu  HTTP=%s  Keys: c/b/r/o/p/t/g/f/u(override)/n/q\n\n",
               (unsigned long)blackbox.capacity, http_ok ? "on" : "off");
    }

    while (g_running) {
        TelemetryPacket pkt;
        Alert alert;
        AlertType atype;
        int sub;
        int key;
        int triggered_in_tick = 0;

        /* --- High-rate 250 Hz IMU sensing + detector loop ---------------- */
        for (sub = 0; sub < CFG_IMU_SUBSTEPS_PER_TICK; sub++) {
            simulator_step_fast(&sim, &pkt);
            atype = detector_process(&detector, &pkt, &alert);

            if (atype != ALERT_NONE) {
                /* Enqueue exact triggering high-rate packet into blackbox */
                cq_enqueue(&blackbox, &pkt);
                triggered_in_tick = 1;
                sm_on_alert(&sm, atype, pkt.timestamp);
                printf("[DET] %s (%s) at t=%.3f lat=%.5f lon=%.5f | %s\n",
                       alert_type_name(atype), severity_name(alert.severity),
                       pkt.timestamp, pkt.latitude, pkt.longitude,
                       detector.last_reason);
                printf("[SM ] State -> %s | %s\n",
                       sm_state_name(sm.state), sm.status_msg);
                reconstruct_timeline(&blackbox, &alert, &pkt);
                publisher_enqueue(&publisher, &alert);
                state.last_alert_type = atype;
                state.last_alert_time = pkt.timestamp;
                if (atype == ALERT_COLLISION || atype == ALERT_ROLLOVER) {
                    state.has_crash_loc = 1;
                    state.crash_lat     = pkt.latitude;
                    state.crash_lon     = pkt.longitude;
                }
            }
        }

        /* --- Slow 5 Hz telemetry tick ------------------------------------ */
        if (!triggered_in_tick)
            cq_enqueue(&blackbox, &pkt); /* O(1) ring write / overwrite */

        state.latest = pkt;
        state.sim_time = sim.t;
        plot_push(&state, &pkt);

        hal_watchdog_feed(sim.t);
        sm_tick(&sm, sim.t, pkt.speed);

        publisher_tick(&publisher, sim.t);

        if (max_duration >= 0.0 && sim.t >= max_duration)
            g_running = 0;

        if (http_ok) {
            http_server_poll(&http);
            http_server_broadcast_sse(&http);
        }

        /* --- keyboard controls ------------------------------------------ */
        key = kb_poll();
        if (key >= 0) {
            switch (key) {
            case 'q': case 'Q': g_running = 0; break;
            case 'n': case 'N':
                publisher_toggle_network(&publisher);
                hal_modem_set_link_up(publisher_network_up(&publisher));
                break;
            case 'c': case 'C': simulator_inject(&sim, SCENARIO_COLLISION); break;
            case 'b': case 'B': simulator_inject(&sim, SCENARIO_HARD_BRAKE); break;
            case 'r': case 'R': simulator_inject(&sim, SCENARIO_ROLLOVER); break;
            case 'o': case 'O': simulator_inject(&sim, SCENARIO_OVERHEAT); break;
            case 'p': case 'P': simulator_inject(&sim, SCENARIO_POTHOLE); break;
            case 't': case 'T':
                detector_set_armed(&detector, 1);
                simulator_inject(&sim, SCENARIO_THEFT);
                break;
            case 'g': case 'G': simulator_inject(&sim, SCENARIO_GPS_LOSS); break;
            case 'f': case 'F': simulator_inject(&sim, SCENARIO_SENSOR_FAULT); break;
            case 'u': case 'U':
                sm_override_pin(&sm, CFG_DEFAULT_OVERRIDE_PIN, sim.t);
                detector_set_armed(&detector, 0);
                printf("[SM ] PIN Override -> %s\n", sm.status_msg);
                break;
            default: break;
            }
        }

        hal_time_sleep_ms(200); /* 5 Hz outer frame (50 x 4ms IMU substeps) */
    }

    printf("\nShutting down...\n");
#ifndef _WIN32
    term_raw_off();
#endif
    if (http_ok)
        http_server_shutdown(&http);
    publisher_shutdown(&publisher);
    cq_free(&blackbox);
    return 0;
}
