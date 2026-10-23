/**
 * hal.h — Hardware Abstraction Layer (HAL) for Telemetry & Anomaly Engine.
 *
 * Provides two backends:
 *   1. `sim` backend (src/hal_sim.c) — default on desktop; 100% hardware-free.
 *   2. `esp32` backend (esp32_firmware/main_car_controller/hal_esp32.h) — real hardware.
 *
 * The core engine (circular_queue, detector, linked_list, state_machine, publisher)
 * calls ONLY these HAL functions and contains zero hardware #ifdefs.
 */
#ifndef HAL_H
#define HAL_H

#include "config.h"
#include "structs.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- IMU Calibration & Mounting Orientation ------------------------------ */
typedef struct {
    double accel_bias[3];      /* Bias offsets (m/s^2) computed at boot       */
    double gyro_bias[3];       /* Gyro offsets (deg/s) computed at boot       */
    int    axis_sign[3];       /* Mounting orientation signs (+1 or -1)       */
    int    axis_map[3];        /* Axis permutation indices {0, 1, 2}          */
    int    calibrated;         /* 1 once boot calibration succeeded           */
    int    remove_gravity;     /* 1 to subtract 1g (9.81 m/s^2) on Z axis     */
} HalImuCalibration;

typedef struct {
    double ax, ay, az;         /* Calibrated acceleration (m/s^2, Z w/ gravity) */
    double lin_ax, lin_ay, lin_az; /* Gravity-removed linear acceleration     */
    double gx, gy, gz;         /* Calibrated angular velocity (deg/s)         */
    double timestamp;          /* Sample timestamp (seconds)                  */
    int    fifo_samples;       /* Number of FIFO samples drained              */
    int    fault;              /* 1 if I2C error or rail saturation detected  */
} HalImuSample;

/* ---- Virtual / Physical Relays ------------------------------------------- */
typedef enum {
    HAL_RELAY_ENGINE = 0,      /* Engine cut-off (0 = run allowed, 1 = cut)   */
    HAL_RELAY_SIREN  = 1,      /* Alarm siren (0 = off, 1 = active)           */
    HAL_RELAY_LIGHTS = 2,      /* Hazard / headlights (0 = off, 1 = flashing) */
    HAL_RELAY_LOCK   = 3,      /* Door lock (0 = unlocked, 1 = locked)        */
    HAL_RELAY_COUNT  = 4
} HalRelayId;

typedef struct {
    double     timestamp;
    HalRelayId relay;
    int        state;
    char       reason[48];
} HalRelayLogEntry;

/* ---- Modem (SIM7670C 4G + GNSS) & LoRa Status ---------------------------- */
typedef struct {
    int    attached;           /* 1 = registered & packet-attached            */
    int    rssi_dbm;           /* Signal strength (-110 .. -50 dBm)           */
    int    gnss_fix;           /* 1 = valid GNSS fix, 0 = lost                */
    double lat, lon;           /* Latest GNSS coordinates                     */
    int    tx_queued;
    int    tx_sent;
    int    tx_failed;
    char   last_at_cmd[64];
    char   last_at_resp[64];
} HalModemStatus;

typedef struct {
    int    initialized;
    int    rssi_dbm;
    int    packets_sent;
    char   last_payload[96];
} HalLoRaStatus;

/* ---- 1. hal_time --------------------------------------------------------- */
double hal_time_now(void);
void   hal_time_sleep_ms(int ms);

/* ---- 2. hal_imu (200-500 Hz high-rate sensing + boot calibration) -------- */
int  hal_imu_init(void);
int  hal_imu_calibrate(HalImuCalibration *cal, int num_samples);
void hal_imu_set_mounting(HalImuCalibration *cal,
                          int map_x, int map_y, int map_z,
                          int sign_x, int sign_y, int sign_z);
void hal_imu_apply_calibration(const HalImuCalibration *cal,
                               double raw_ax, double raw_ay, double raw_az,
                               double raw_gx, double raw_gy, double raw_gz,
                               HalImuSample *out);
int  hal_imu_read_sample(HalImuSample *out);
const HalImuCalibration *hal_imu_get_calibration(void);

/* ---- 3. hal_temp (LM35 engine temperature + battery voltage) ------------- */
double hal_temp_read_celsius(void);
double hal_battery_read_pct(void);

/* ---- 4. hal_speed_gps (Wheel speed + GPS coordinates) -------------------- */
double hal_speed_read_kmh(void);
int    hal_gps_read(double *lat, double *lon);

/* ---- 5. hal_relay (Fail-safe relay control + actuation log) -------------- */
void hal_relay_init_failsafe(void);
void hal_relay_set(HalRelayId id, int state, const char *reason);
int  hal_relay_get(HalRelayId id);
const char *hal_relay_name(HalRelayId id);
size_t hal_relay_get_log(HalRelayLogEntry *out, size_t max_entries);

/* ---- 6. hal_modem (SIM7670C 4G + GNSS AT command transport) -------------- */
int  hal_modem_init(void);
void hal_modem_set_link_up(int up);
int  hal_modem_is_link_up(void);
int  hal_modem_send_at(const char *cmd, char *resp, size_t resp_len, int timeout_ms);
int  hal_modem_publish_alert(const Alert *alert, const char *signature);
void hal_modem_get_status(HalModemStatus *out);

/* ---- 7. hal_lora (Car-to-car / emergency fallback radio) ----------------- */
int  hal_lora_init(void);
int  hal_lora_send_alert(const Alert *alert);
void hal_lora_get_status(HalLoRaStatus *out);

/* ---- 8. hal_storage (Batched LittleFS / disk persistence) ---------------- */
int  hal_storage_init(void);
int  hal_storage_write_file(const char *path, const char *data, size_t len);
int  hal_storage_append_file(const char *path, const char *data, size_t len);
int  hal_storage_read_file(const char *path, char *buf, size_t max_len, size_t *out_len);

/* ---- 9. Watchdog --------------------------------------------------------- */
void hal_watchdog_init(double timeout_sec);
void hal_watchdog_feed(double now_sec);
int  hal_watchdog_check(double now_sec); /* returns 1 if expired -> triggers fail-safe */

#ifdef __cplusplus
}
#endif

#endif /* HAL_H */
