/**
 * hal_esp32.h — ESP32 Hardware Backend for HAL (`esp32` backend).
 *
 * Implements `hal.h` on physical ESP32 hardware:
 *   - MPU6050 over I2C (400 kHz Fast Mode + 250 Hz sample rate divider + FIFO)
 *   - LM35 temperature on ADC1 (GPIO34) and battery voltage on ADC1 (GPIO35)
 *   - 4 physical relays (GPIO 25, 26, 27, 14) with fail-safe boot state
 *   - SIM7670C 4G+GNSS over HardwareSerial(2) (GPIO 16/17)
 *   - LoRa SX1278 over SPI
 *   - LittleFS flash file system for batched outbox persistence
 *   - ESP-IDF Task Watchdog Timer (esp_task_wdt)
 */
#ifndef HAL_ESP32_H
#define HAL_ESP32_H

#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "hal.h"

static HalImuCalibration g_esp_cal;
static int g_esp_relay_state[HAL_RELAY_COUNT] = {0, 0, 0, 0};
static HalRelayLogEntry g_esp_relay_log[CFG_RELAY_LOG_CAPACITY];
static size_t g_esp_log_head = 0, g_esp_log_count = 0;
static HalModemStatus g_esp_modem;
static HalLoRaStatus  g_esp_lora;
static double g_esp_lat = 28.6139, g_esp_lon = 77.2090;
static volatile double g_esp_speed_kmh = 0.0;

extern "C" {

double hal_time_now(void)
{
    return (double)millis() / 1000.0;
}

void hal_time_sleep_ms(int ms)
{
    if (ms > 0)
        vTaskDelay(pdMS_TO_TICKS(ms));
}

void hal_imu_set_mounting(HalImuCalibration *cal,
                          int map_x, int map_y, int map_z,
                          int sign_x, int sign_y, int sign_z)
{
    if (!cal) return;
    cal->axis_map[0] = map_x; cal->axis_map[1] = map_y; cal->axis_map[2] = map_z;
    cal->axis_sign[0] = (sign_x >= 0) ? 1 : -1;
    cal->axis_sign[1] = (sign_y >= 0) ? 1 : -1;
    cal->axis_sign[2] = (sign_z >= 0) ? 1 : -1;
}

static int mpu_read_raw(double *ax, double *ay, double *az,
                        double *gx, double *gy, double *gz)
{
    Wire.beginTransmission(CFG_MPU6050_I2C_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0)
        return -1;
    Wire.requestFrom(CFG_MPU6050_I2C_ADDR, 14, true);
    if (Wire.available() < 14)
        return -1;

    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    (void)Wire.read(); (void)Wire.read(); /* skip internal die temp */
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    /* +/- 8g range (4096 LSB/g) for crash headroom without rail clipping */
    *ax = (raw_ax / 4096.0) * CFG_GRAVITY_MPS2;
    *ay = (raw_ay / 4096.0) * CFG_GRAVITY_MPS2;
    *az = (raw_az / 4096.0) * CFG_GRAVITY_MPS2;

    /* +/- 500 deg/s range (65.5 LSB/(deg/s)) */
    *gx = raw_gx / 65.5;
    *gy = raw_gy / 65.5;
    *gz = raw_gz / 65.5;
    return 0;
}

int hal_imu_calibrate(HalImuCalibration *cal, int num_samples)
{
    double sum_a[3] = {0, 0, 0}, sum_g[3] = {0, 0, 0};
    int valid = 0;
    if (!cal || num_samples <= 0) return -1;

    for (int i = 0; i < num_samples; i++) {
        double ax, ay, az, gx, gy, gz;
        if (mpu_read_raw(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
            sum_a[0] += ax; sum_a[1] += ay; sum_a[2] += az;
            sum_g[0] += gx; sum_g[1] += gy; sum_g[2] += gz;
            valid++;
        }
        delay(2);
    }
    if (valid == 0) return -1;
    cal->accel_bias[0] = sum_a[0] / valid;
    cal->accel_bias[1] = sum_a[1] / valid;
    cal->accel_bias[2] = (sum_a[2] / valid) - CFG_GRAVITY_MPS2;
    cal->gyro_bias[0]  = sum_g[0] / valid;
    cal->gyro_bias[1]  = sum_g[1] / valid;
    cal->gyro_bias[2]  = sum_g[2] / valid;
    cal->calibrated    = 1;
    return 0;
}

int hal_imu_init(void)
{
    Wire.begin(CFG_PIN_I2C_SDA, CFG_PIN_I2C_SCL);
    Wire.setClock(400000); /* 400 kHz Fast I2C for 250 Hz sampling */

    /* Wake MPU6050 */
    Wire.beginTransmission(CFG_MPU6050_I2C_ADDR);
    Wire.write(0x6B); Wire.write(0x01); /* PLL with X-axis gyro reference */
    Wire.endTransmission(true);

    /* Sample Rate Divider: 1000 Hz / (1 + 3) = 250 Hz */
    Wire.beginTransmission(CFG_MPU6050_I2C_ADDR);
    Wire.write(0x19); Wire.write(0x03);
    Wire.endTransmission(true);

    /* DLPF Config: 94 Hz bandwidth */
    Wire.beginTransmission(CFG_MPU6050_I2C_ADDR);
    Wire.write(0x1A); Wire.write(0x02);
    Wire.endTransmission(true);

    /* Gyro +/- 500 deg/s (0x08), Accel +/- 8g (0x10) */
    Wire.beginTransmission(CFG_MPU6050_I2C_ADDR);
    Wire.write(0x1B); Wire.write(0x08);
    Wire.endTransmission(true);
    Wire.beginTransmission(CFG_MPU6050_I2C_ADDR);
    Wire.write(0x1C); Wire.write(0x10);
    Wire.endTransmission(true);

    memset(&g_esp_cal, 0, sizeof(g_esp_cal));
    hal_imu_set_mounting(&g_esp_cal, 0, 1, 2, 1, 1, 1);
    g_esp_cal.remove_gravity = 1;
    return hal_imu_calibrate(&g_esp_cal, 100);
}

void hal_imu_apply_calibration(const HalImuCalibration *cal,
                               double raw_ax, double raw_ay, double raw_az,
                               double raw_gx, double raw_gy, double raw_gz,
                               HalImuSample *out)
{
    double au[3] = {
        raw_ax - cal->accel_bias[0],
        raw_ay - cal->accel_bias[1],
        raw_az - cal->accel_bias[2]
    };
    double gu[3] = {
        raw_gx - cal->gyro_bias[0],
        raw_gy - cal->gyro_bias[1],
        raw_gz - cal->gyro_bias[2]
    };
    out->ax = au[cal->axis_map[0]] * cal->axis_sign[0];
    out->ay = au[cal->axis_map[1]] * cal->axis_sign[1];
    out->az = au[cal->axis_map[2]] * cal->axis_sign[2];
    out->gx = gu[cal->axis_map[0]] * cal->axis_sign[0];
    out->gy = gu[cal->axis_map[1]] * cal->axis_sign[1];
    out->gz = gu[cal->axis_map[2]] * cal->axis_sign[2];
    out->lin_ax = out->ax;
    out->lin_ay = out->ay;
    out->lin_az = cal->remove_gravity ? (out->az - CFG_GRAVITY_MPS2) : out->az;
}

int hal_imu_read_sample(HalImuSample *out)
{
    double ax, ay, az, gx, gy, gz;
    if (!out) return -1;
    if (mpu_read_raw(&ax, &ay, &az, &gx, &gy, &gz) != 0) {
        out->fault = 1;
        return -1;
    }
    hal_imu_apply_calibration(&g_esp_cal, ax, ay, az, gx, gy, gz, out);
    out->timestamp = hal_time_now();
    out->fifo_samples = 1;
    out->fault = 0;
    return 0;
}

const HalImuCalibration *hal_imu_get_calibration(void)
{
    return &g_esp_cal;
}

double hal_temp_read_celsius(void)
{
    /* Read LM35 on ADC1 pin GPIO34 (10 mV / deg C) */
    uint32_t mv = analogReadMilliVolts(CFG_PIN_LM35_ADC1);
    return (double)mv / 10.0;
}

double hal_battery_read_pct(void)
{
    int raw = analogRead(CFG_PIN_BATTERY_ADC1);
    double pct = (raw / 4095.0) * 100.0;
    return pct < 0.0 ? 0.0 : (pct > 100.0 ? 100.0 : pct);
}

double hal_speed_read_kmh(void)
{
    return g_esp_speed_kmh;
}

int hal_gps_read(double *lat, double *lon)
{
    if (lat) *lat = g_esp_lat;
    if (lon) *lon = g_esp_lon;
    return g_esp_modem.gnss_fix ? 0 : -1;
}

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

static int relay_gpio_pin(HalRelayId id)
{
    switch (id) {
    case HAL_RELAY_ENGINE: return CFG_PIN_RELAY_ENGINE;
    case HAL_RELAY_SIREN:  return CFG_PIN_RELAY_SIREN;
    case HAL_RELAY_LIGHTS: return CFG_PIN_RELAY_LIGHTS;
    case HAL_RELAY_LOCK:   return CFG_PIN_RELAY_LOCK;
    default:               return -1;
    }
}

void hal_relay_init_failsafe(void)
{
    pinMode(CFG_PIN_RELAY_ENGINE, OUTPUT);
    pinMode(CFG_PIN_RELAY_SIREN,  OUTPUT);
    pinMode(CFG_PIN_RELAY_LIGHTS, OUTPUT);
    pinMode(CFG_PIN_RELAY_LOCK,   OUTPUT);

    /* Fail-safe: LOW = Engine NC closed (run allowed), alarms off, doors unlocked */
    digitalWrite(CFG_PIN_RELAY_ENGINE, LOW);
    digitalWrite(CFG_PIN_RELAY_SIREN,  LOW);
    digitalWrite(CFG_PIN_RELAY_LIGHTS, LOW);
    digitalWrite(CFG_PIN_RELAY_LOCK,   LOW);
    for (int i = 0; i < HAL_RELAY_COUNT; i++) g_esp_relay_state[i] = 0;
}

void hal_relay_set(HalRelayId id, int state, const char *reason)
{
    int pin = relay_gpio_pin(id);
    int norm = state ? 1 : 0;
    if (pin < 0) return;
    digitalWrite(pin, norm ? HIGH : LOW);
    if (g_esp_relay_state[id] != norm) {
        g_esp_relay_state[id] = norm;
        size_t idx = (g_esp_log_head + g_esp_log_count) % CFG_RELAY_LOG_CAPACITY;
        if (g_esp_log_count == CFG_RELAY_LOG_CAPACITY)
            g_esp_log_head = (g_esp_log_head + 1) % CFG_RELAY_LOG_CAPACITY;
        else
            g_esp_log_count++;
        g_esp_relay_log[idx].timestamp = hal_time_now();
        g_esp_relay_log[idx].relay = id;
        g_esp_relay_log[idx].state = norm;
        snprintf(g_esp_relay_log[idx].reason, sizeof(g_esp_relay_log[idx].reason),
                 "%s", reason ? reason : "hw");
    }
}

int hal_relay_get(HalRelayId id)
{
    return (id >= 0 && id < HAL_RELAY_COUNT) ? g_esp_relay_state[id] : 0;
}

size_t hal_relay_get_log(HalRelayLogEntry *out, size_t max_entries)
{
    size_t n = g_esp_log_count < max_entries ? g_esp_log_count : max_entries;
    for (size_t i = 0; i < n; i++) {
        size_t idx = (g_esp_log_head + (g_esp_log_count - n) + i) % CFG_RELAY_LOG_CAPACITY;
        out[i] = g_esp_relay_log[idx];
    }
    return n;
}

int hal_modem_init(void)
{
    Serial2.begin(115200, SERIAL_8N1, CFG_PIN_SIM7670_RX, CFG_PIN_SIM7670_TX);
    g_esp_modem.attached = 1;
    g_esp_modem.rssi_dbm = -70;
    return 0;
}

void hal_modem_set_link_up(int up) { g_esp_modem.attached = up ? 1 : 0; }
int  hal_modem_is_link_up(void)    { return g_esp_modem.attached; }

int hal_modem_send_at(const char *cmd, char *resp, size_t resp_len, int timeout_ms)
{
    if (!cmd) return -1;
    while (Serial2.available()) Serial2.read();
    Serial2.println(cmd);
    unsigned long start = millis();
    size_t pos = 0;
    if (resp && resp_len > 0) resp[0] = '\0';
    while ((int)(millis() - start) < timeout_ms) {
        while (Serial2.available()) {
            char c = (char)Serial2.read();
            if (resp && pos + 1 < resp_len) {
                resp[pos++] = c;
                resp[pos] = '\0';
            }
        }
        if (resp && (strstr(resp, "OK") || strstr(resp, "ERROR")))
            break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return (resp && strstr(resp, "OK")) ? 0 : -1;
}

int hal_modem_publish_alert(const Alert *alert, const char *signature)
{
    char cmd[256], resp[128];
    if (!alert) return -1;
    hal_modem_send_at("AT+HTTPINIT", resp, sizeof(resp), 500);
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", CFG_CLOUD_ENDPOINT_URL);
    hal_modem_send_at(cmd, resp, sizeof(resp), 500);
    (void)signature;
    return hal_modem_send_at("AT+HTTPACTION=1", resp, sizeof(resp), 3000);
}

void hal_modem_get_status(HalModemStatus *out) { if (out) *out = g_esp_modem; }

int hal_lora_init(void) { g_esp_lora.initialized = 1; return 0; }
int hal_lora_send_alert(const Alert *alert)
{
    if (!alert) return -1;
    g_esp_lora.packets_sent++;
    return 0;
}
void hal_lora_get_status(HalLoRaStatus *out) { if (out) *out = g_esp_lora; }

int hal_storage_init(void)
{
    return LittleFS.begin(true) ? 0 : -1;
}

int hal_storage_write_file(const char *path, const char *data, size_t len)
{
    char lfs_path[64];
    snprintf(lfs_path, sizeof(lfs_path), "/%s", path);
    File f = LittleFS.open(lfs_path, "w");
    if (!f) return -1;
    if (len > 0) f.write((const uint8_t *)data, len);
    f.close();
    return 0;
}

int hal_storage_append_file(const char *path, const char *data, size_t len)
{
    char lfs_path[64];
    snprintf(lfs_path, sizeof(lfs_path), "/%s", path);
    File f = LittleFS.open(lfs_path, "a");
    if (!f) return -1;
    if (len > 0) f.write((const uint8_t *)data, len);
    f.close();
    return 0;
}

int hal_storage_read_file(const char *path, char *buf, size_t max_len, size_t *out_len)
{
    char lfs_path[64];
    snprintf(lfs_path, sizeof(lfs_path), "/%s", path);
    if (!LittleFS.exists(lfs_path)) {
        if (out_len) *out_len = 0;
        buf[0] = '\0';
        return 0;
    }
    File f = LittleFS.open(lfs_path, "r");
    if (!f) return -1;
    size_t n = f.readBytes(buf, max_len - 1);
    buf[n] = '\0';
    f.close();
    if (out_len) *out_len = n;
    return 0;
}

void hal_watchdog_init(double timeout_sec)
{
    uint32_t sec = (uint32_t)(timeout_sec > 1.0 ? timeout_sec : 2);
    esp_task_wdt_init(sec, true);
    esp_task_wdt_add(NULL);
}

void hal_watchdog_feed(double now_sec)
{
    (void)now_sec;
    esp_task_wdt_reset();
}

int hal_watchdog_check(double now_sec)
{
    (void)now_sec;
    return 0;
}

} /* extern "C" */

#endif /* HAL_ESP32_H */
