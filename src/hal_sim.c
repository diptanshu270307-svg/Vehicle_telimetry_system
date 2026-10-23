/**
 * hal_sim.c — Desktop Hardware-Free (`sim`) backend for HAL.
 *
 * Emulates MPU6050 (with realistic bias + gravity removal calibration),
 * LM35 temperature, wheel speed + GNSS, 4 automotive relays with fail-safe
 * state and actuation ring-log, SIM7670C AT-command modem, LoRa transceiver,
 * disk storage, and hardware watchdog.
 */
#include "hal.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  include <windows.h>
#  include <direct.h>
#else
#  include <time.h>
#  include <sys/stat.h>
#endif

/* ---- Simulated Hardware State -------------------------------------------- */
static double g_sim_clock = 0.0;
static HalImuCalibration g_imu_cal;

/* Simulated raw sensor biases (measured & canceled at boot by hal_imu_calibrate) */
static const double SIM_RAW_ACCEL_BIAS[3] = { 0.14, -0.09, 0.22 };
static const double SIM_RAW_GYRO_BIAS[3]  = { 0.42, -0.31, 0.18 };

/* Latest physical values fed by simulator */
static double g_latest_raw_ax = 0.0;
static double g_latest_raw_ay = 0.0;
static double g_latest_raw_az = CFG_GRAVITY_MPS2;
static double g_latest_raw_gx = 0.0;
static double g_latest_raw_gy = 0.0;
static double g_latest_raw_gz = 0.0;
static double g_latest_temp_c = 85.0;
static double g_latest_batt   = 78.0;
static double g_latest_speed  = 60.0;
static double g_latest_lat    = 28.6139;
static double g_latest_lon    = 77.2090;
static int    g_gps_valid     = 1;
static int    g_imu_fault     = 0;

/* Relay state & actuation ring buffer */
static int g_relay_state[HAL_RELAY_COUNT] = { 0, 0, 0, 0 };
static HalRelayLogEntry g_relay_log[CFG_RELAY_LOG_CAPACITY];
static size_t g_relay_log_head  = 0;
static size_t g_relay_log_count = 0;

/* Virtual SIM7670C & LoRa state */
static HalModemStatus g_modem;
static HalLoRaStatus  g_lora;

/* Watchdog state */
static double g_wdt_timeout   = CFG_SM_WATCHDOG_TIMEOUT_S;
static double g_wdt_last_feed = 0.0;
static int    g_wdt_enabled   = 0;

/* ---- Simulation injection helpers (used by simulator.c) ------------------ */
void hal_sim_set_clock(double t)
{
    g_sim_clock = t;
}

void hal_sim_feed_sensors(double ax, double ay, double az,
                          double gx, double gy, double gz,
                          double speed_kmh, double temp_c, double battery_pct,
                          double lat, double lon, int gps_valid, int imu_fault)
{
    /* Add raw hardware bias so calibration pipeline is actively exercised */
    g_latest_raw_ax = ax + SIM_RAW_ACCEL_BIAS[0];
    g_latest_raw_ay = ay + SIM_RAW_ACCEL_BIAS[1];
    g_latest_raw_az = az + SIM_RAW_ACCEL_BIAS[2];
    g_latest_raw_gx = gx + SIM_RAW_GYRO_BIAS[0];
    g_latest_raw_gy = gy + SIM_RAW_GYRO_BIAS[1];
    g_latest_raw_gz = gz + SIM_RAW_GYRO_BIAS[2];
    g_latest_speed  = speed_kmh;
    g_latest_temp_c = temp_c;
    g_latest_batt   = battery_pct;
    g_latest_lat    = lat;
    g_latest_lon    = lon;
    g_gps_valid     = gps_valid;
    g_imu_fault     = imu_fault;
    g_modem.gnss_fix = gps_valid;
    if (gps_valid) {
        g_modem.lat = lat;
        g_modem.lon = lon;
    }
}

/* ---- 1. hal_time --------------------------------------------------------- */
double hal_time_now(void)
{
    return g_sim_clock;
}

void hal_time_sleep_ms(int ms)
{
    if (ms <= 0)
        return;
#ifdef _WIN32
    Sleep((DWORD)ms);
#else
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
#endif
}

/* ---- 2. hal_imu ---------------------------------------------------------- */
int hal_imu_init(void)
{
    memset(&g_imu_cal, 0, sizeof(g_imu_cal));
    g_imu_cal.axis_map[0] = 0;
    g_imu_cal.axis_map[1] = 1;
    g_imu_cal.axis_map[2] = 2;
    g_imu_cal.axis_sign[0] = 1;
    g_imu_cal.axis_sign[1] = 1;
    g_imu_cal.axis_sign[2] = 1;
    g_imu_cal.remove_gravity = 1;
    return hal_imu_calibrate(&g_imu_cal, 64);
}

void hal_imu_set_mounting(HalImuCalibration *cal,
                          int map_x, int map_y, int map_z,
                          int sign_x, int sign_y, int sign_z)
{
    if (!cal)
        return;
    if (map_x >= 0 && map_x <= 2) cal->axis_map[0] = map_x;
    if (map_y >= 0 && map_y <= 2) cal->axis_map[1] = map_y;
    if (map_z >= 0 && map_z <= 2) cal->axis_map[2] = map_z;
    cal->axis_sign[0] = (sign_x >= 0) ? 1 : -1;
    cal->axis_sign[1] = (sign_y >= 0) ? 1 : -1;
    cal->axis_sign[2] = (sign_z >= 0) ? 1 : -1;
}

int hal_imu_calibrate(HalImuCalibration *cal, int num_samples)
{
    int i;
    double sum_a[3] = {0.0, 0.0, 0.0};
    double sum_g[3] = {0.0, 0.0, 0.0};

    if (!cal || num_samples <= 0)
        return -1;

    /* Average stationary samples at boot (vehicle flat: ax=0, ay=0, az=+9.81) */
    for (i = 0; i < num_samples; i++) {
        double jitter = ((i % 5) - 2) * 0.002;
        sum_a[0] += SIM_RAW_ACCEL_BIAS[0] + jitter;
        sum_a[1] += SIM_RAW_ACCEL_BIAS[1] - jitter;
        sum_a[2] += CFG_GRAVITY_MPS2 + SIM_RAW_ACCEL_BIAS[2] + jitter;
        sum_g[0] += SIM_RAW_GYRO_BIAS[0] + jitter;
        sum_g[1] += SIM_RAW_GYRO_BIAS[1] - jitter;
        sum_g[2] += SIM_RAW_GYRO_BIAS[2] + jitter;
    }

    cal->accel_bias[0] = sum_a[0] / (double)num_samples;
    cal->accel_bias[1] = sum_a[1] / (double)num_samples;
    cal->accel_bias[2] = (sum_a[2] / (double)num_samples) - CFG_GRAVITY_MPS2;
    cal->gyro_bias[0]  = sum_g[0] / (double)num_samples;
    cal->gyro_bias[1]  = sum_g[1] / (double)num_samples;
    cal->gyro_bias[2]  = sum_g[2] / (double)num_samples;
    cal->calibrated    = 1;

    /* Copy into active global calibration if calibrating a custom struct */
    if (cal != &g_imu_cal)
        g_imu_cal = *cal;

    return 0;
}

void hal_imu_apply_calibration(const HalImuCalibration *cal,
                               double raw_ax, double raw_ay, double raw_az,
                               double raw_gx, double raw_gy, double raw_gz,
                               HalImuSample *out)
{
    double a_unbiased[3], g_unbiased[3];
    double a_mapped[3], g_mapped[3];
    int i;

    if (!out)
        return;
    memset(out, 0, sizeof(*out));

    if (!cal || !cal->calibrated) {
        out->ax = raw_ax;
        out->ay = raw_ay;
        out->az = raw_az;
        out->lin_ax = raw_ax;
        out->lin_ay = raw_ay;
        out->lin_az = raw_az - CFG_GRAVITY_MPS2;
        out->gx = raw_gx;
        out->gy = raw_gy;
        out->gz = raw_gz;
        return;
    }

    /* 1. Subtract boot-calibrated sensor bias offsets */
    a_unbiased[0] = raw_ax - cal->accel_bias[0];
    a_unbiased[1] = raw_ay - cal->accel_bias[1];
    a_unbiased[2] = raw_az - cal->accel_bias[2];

    g_unbiased[0] = raw_gx - cal->gyro_bias[0];
    g_unbiased[1] = raw_gy - cal->gyro_bias[1];
    g_unbiased[2] = raw_gz - cal->gyro_bias[2];

    /* 2. Apply mounting orientation axis permutation and sign inversion */
    for (i = 0; i < 3; i++) {
        int src = cal->axis_map[i];
        int sgn = cal->axis_sign[i];
        if (src < 0 || src > 2) src = i;
        a_mapped[i] = a_unbiased[src] * (double)sgn;
        g_mapped[i] = g_unbiased[src] * (double)sgn;
    }

    out->ax = a_mapped[0];
    out->ay = a_mapped[1];
    out->az = a_mapped[2];
    out->gx = g_mapped[0];
    out->gy = g_mapped[1];
    out->gz = g_mapped[2];

    /* 3. Gravity removal for linear acceleration components */
    out->lin_ax = out->ax;
    out->lin_ay = out->ay;
    out->lin_az = cal->remove_gravity ? (out->az - CFG_GRAVITY_MPS2) : out->az;
}

int hal_imu_read_sample(HalImuSample *out)
{
    if (!out)
        return -1;
    hal_imu_apply_calibration(&g_imu_cal,
                              g_latest_raw_ax, g_latest_raw_ay, g_latest_raw_az,
                              g_latest_raw_gx, g_latest_raw_gy, g_latest_raw_gz,
                              out);
    out->timestamp    = g_sim_clock;
    out->fifo_samples = 1;
    out->fault        = g_imu_fault;
    return g_imu_fault ? -1 : 0;
}

const HalImuCalibration *hal_imu_get_calibration(void)
{
    return &g_imu_cal;
}

/* ---- 3. hal_temp & battery ----------------------------------------------- */
double hal_temp_read_celsius(void)
{
    return g_latest_temp_c;
}

double hal_battery_read_pct(void)
{
    return g_latest_batt;
}

/* ---- 4. hal_speed_gps ---------------------------------------------------- */
double hal_speed_read_kmh(void)
{
    return g_latest_speed;
}

int hal_gps_read(double *lat, double *lon)
{
    if (lat) *lat = g_latest_lat;
    if (lon) *lon = g_latest_lon;
    return g_gps_valid ? 0 : -1;
}

/* ---- 5. hal_relay -------------------------------------------------------- */
const char *hal_relay_name(HalRelayId id)
{
    switch (id) {
    case HAL_RELAY_ENGINE: return "ENGINE_CUT";
    case HAL_RELAY_SIREN:  return "SIREN";
    case HAL_RELAY_LIGHTS: return "HAZARD_LIGHTS";
    case HAL_RELAY_LOCK:   return "DOOR_LOCK";
    default:               return "UNKNOWN";
    }
}

static void log_relay_transition(HalRelayId id, int state, const char *reason)
{
    size_t idx = (g_relay_log_head + g_relay_log_count) % CFG_RELAY_LOG_CAPACITY;
    if (g_relay_log_count == CFG_RELAY_LOG_CAPACITY) {
        g_relay_log_head = (g_relay_log_head + 1) % CFG_RELAY_LOG_CAPACITY;
    } else {
        g_relay_log_count++;
    }
    g_relay_log[idx].timestamp = g_sim_clock;
    g_relay_log[idx].relay     = id;
    g_relay_log[idx].state     = state;
    snprintf(g_relay_log[idx].reason, sizeof(g_relay_log[idx].reason),
             "%s", reason ? reason : "manual");
}

void hal_relay_init_failsafe(void)
{
    int i;
    /*
     * Fail-safe states on boot / brown-out / watchdog reset:
     *   - ENGINE_CUT = 0 (NC relay unenergized -> engine allowed to run safely)
     *   - SIREN      = 0 (off)
     *   - LIGHTS     = 0 (off)
     *   - DOOR_LOCK  = 0 (unlocked so occupants are never trapped on power fault)
     */
    for (i = 0; i < HAL_RELAY_COUNT; i++) {
        g_relay_state[i] = 0;
    }
    log_relay_transition(HAL_RELAY_ENGINE, 0, "boot_failsafe");
}

void hal_relay_set(HalRelayId id, int state, const char *reason)
{
    int norm = state ? 1 : 0;
    if (id < 0 || id >= HAL_RELAY_COUNT)
        return;
    if (g_relay_state[id] != norm) {
        g_relay_state[id] = norm;
        log_relay_transition(id, norm, reason);
    }
}

int hal_relay_get(HalRelayId id)
{
    if (id < 0 || id >= HAL_RELAY_COUNT)
        return 0;
    return g_relay_state[id];
}

size_t hal_relay_get_log(HalRelayLogEntry *out, size_t max_entries)
{
    size_t i, n;
    if (!out || max_entries == 0)
        return 0;
    n = g_relay_log_count < max_entries ? g_relay_log_count : max_entries;
    for (i = 0; i < n; i++) {
        size_t idx = (g_relay_log_head + (g_relay_log_count - n) + i) % CFG_RELAY_LOG_CAPACITY;
        out[i] = g_relay_log[idx];
    }
    return n;
}

/* ---- 6. hal_modem (SIM7670C 4G + GNSS) ----------------------------------- */
int hal_modem_init(void)
{
    memset(&g_modem, 0, sizeof(g_modem));
    g_modem.attached = 1;
    g_modem.rssi_dbm = -67;
    g_modem.gnss_fix = 1;
    g_modem.lat      = g_latest_lat;
    g_modem.lon      = g_latest_lon;
    snprintf(g_modem.last_at_cmd, sizeof(g_modem.last_at_cmd), "AT+CGATT=1");
    snprintf(g_modem.last_at_resp, sizeof(g_modem.last_at_resp), "OK (4G LTE Attached)");
    return 0;
}

void hal_modem_set_link_up(int up)
{
    g_modem.attached = up ? 1 : 0;
    g_modem.rssi_dbm = up ? -67 : -113;
    snprintf(g_modem.last_at_cmd, sizeof(g_modem.last_at_cmd), "AT+CREG?");
    snprintf(g_modem.last_at_resp, sizeof(g_modem.last_at_resp),
             up ? "+CREG: 0,1 OK" : "+CREG: 0,0 NO CARRIER");
}

int hal_modem_is_link_up(void)
{
    return g_modem.attached;
}

int hal_modem_send_at(const char *cmd, char *resp, size_t resp_len, int timeout_ms)
{
    (void)timeout_ms;
    if (!cmd)
        return -1;
    snprintf(g_modem.last_at_cmd, sizeof(g_modem.last_at_cmd), "%s", cmd);

    if (!g_modem.attached && strncmp(cmd, "AT+HTTP", 7) == 0) {
        snprintf(g_modem.last_at_resp, sizeof(g_modem.last_at_resp), "ERROR: NO CARRIER");
        if (resp && resp_len)
            snprintf(resp, resp_len, "%s", g_modem.last_at_resp);
        return -1;
    }

    if (strncmp(cmd, "AT+CGNSSINFO", 12) == 0) {
        if (g_modem.gnss_fix) {
            snprintf(g_modem.last_at_resp, sizeof(g_modem.last_at_resp),
                     "+CGNSSINFO: 3,09,%.6f,N,%.6f,E", g_latest_lat, g_latest_lon);
        } else {
            snprintf(g_modem.last_at_resp, sizeof(g_modem.last_at_resp),
                     "+CGNSSINFO: ,,,,,,");
        }
    } else if (strncmp(cmd, "AT+CSQ", 6) == 0) {
        snprintf(g_modem.last_at_resp, sizeof(g_modem.last_at_resp),
                 "+CSQ: %d,99 OK", g_modem.attached ? 23 : 99);
    } else if (strncmp(cmd, "AT+HTTPACTION=1", 15) == 0) {
        snprintf(g_modem.last_at_resp, sizeof(g_modem.last_at_resp),
                 "+HTTPACTION: 1,200,18 OK");
    } else {
        snprintf(g_modem.last_at_resp, sizeof(g_modem.last_at_resp), "OK");
    }

    if (resp && resp_len)
        snprintf(resp, resp_len, "%s", g_modem.last_at_resp);
    return 0;
}

int hal_modem_publish_alert(const Alert *alert, const char *signature)
{
    char resp[64];
    if (!alert)
        return -1;
    if (!g_modem.attached) {
        g_modem.tx_failed++;
        snprintf(g_modem.last_at_cmd, sizeof(g_modem.last_at_cmd),
                 "AT+HTTPACTION=1 (alert #%d)", alert->id);
        snprintf(g_modem.last_at_resp, sizeof(g_modem.last_at_resp),
                 "ERROR: NETWORK DOWN");
        return -1;
    }
    hal_modem_send_at("AT+HTTPINIT", resp, sizeof(resp), 200);
    snprintf(g_modem.last_at_cmd, sizeof(g_modem.last_at_cmd),
             "POST /alerts id=%d sig=%.8s", alert->id, signature ? signature : "none");
    snprintf(g_modem.last_at_resp, sizeof(g_modem.last_at_resp),
             "+HTTPACTION: 1,200 OK");
    g_modem.tx_sent++;
    return 0;
}

void hal_modem_get_status(HalModemStatus *out)
{
    if (out)
        *out = g_modem;
}

/* ---- 7. hal_lora --------------------------------------------------------- */
int hal_lora_init(void)
{
    memset(&g_lora, 0, sizeof(g_lora));
    g_lora.initialized = 1;
    g_lora.rssi_dbm    = -82;
    snprintf(g_lora.last_payload, sizeof(g_lora.last_payload), "IDLE (433MHz SF7)");
    return 0;
}

int hal_lora_send_alert(const Alert *alert)
{
    if (!alert || !g_lora.initialized)
        return -1;
    g_lora.packets_sent++;
    snprintf(g_lora.last_payload, sizeof(g_lora.last_payload),
             "LORA_SOS|id=%d|type=%s|sev=%s|%.5f,%.5f",
             alert->id, alert_type_name(alert->type),
             severity_name(alert->severity), alert->lat, alert->lon);
    return 0;
}

void hal_lora_get_status(HalLoRaStatus *out)
{
    if (out)
        *out = g_lora;
}

/* ---- 8. hal_storage ------------------------------------------------------ */
int hal_storage_init(void)
{
#ifdef _WIN32
    _mkdir("data");
#else
    mkdir("data", 0755);
#endif
    return 0;
}

int hal_storage_write_file(const char *path, const char *data, size_t len)
{
    FILE *fp;
    if (!path || (!data && len > 0))
        return -1;
    fp = fopen(path, "wb");
    if (!fp)
        return -1;
    if (len > 0 && fwrite(data, 1, len, fp) != len) {
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

int hal_storage_append_file(const char *path, const char *data, size_t len)
{
    FILE *fp;
    if (!path || (!data && len > 0))
        return -1;
    fp = fopen(path, "ab");
    if (!fp)
        return -1;
    if (len > 0 && fwrite(data, 1, len, fp) != len) {
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

int hal_storage_read_file(const char *path, char *buf, size_t max_len, size_t *out_len)
{
    FILE *fp;
    size_t n;
    if (!path || !buf || max_len == 0)
        return -1;
    fp = fopen(path, "rb");
    if (!fp) {
        if (out_len) *out_len = 0;
        buf[0] = '\0';
        return 0;
    }
    n = fread(buf, 1, max_len - 1, fp);
    buf[n] = '\0';
    fclose(fp);
    if (out_len)
        *out_len = n;
    return 0;
}

/* ---- 9. Watchdog --------------------------------------------------------- */
void hal_watchdog_init(double timeout_sec)
{
    g_wdt_timeout   = (timeout_sec > 0.0) ? timeout_sec : CFG_SM_WATCHDOG_TIMEOUT_S;
    g_wdt_last_feed = g_sim_clock;
    g_wdt_enabled   = 1;
}

void hal_watchdog_feed(double now_sec)
{
    g_wdt_last_feed = now_sec;
}

int hal_watchdog_check(double now_sec)
{
    if (!g_wdt_enabled)
        return 0;
    if ((now_sec - g_wdt_last_feed) > g_wdt_timeout) {
        /* Watchdog expired -> force all relays to safe state */
        hal_relay_init_failsafe();
        log_relay_transition(HAL_RELAY_ENGINE, 0, "watchdog_reset");
        g_wdt_last_feed = now_sec;
        return 1;
    }
    return 0;
}
