/**
 * config.h — Central configuration constants, thresholds, and hardware placeholders.
 *
 * Pure C11 header shared by desktop simulator and ESP32 firmware.
 * Secrets are NEVER stored here; use environment variables or local overrides.
 */
#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Sampling & Timing Rates --------------------------------------------- */
#define CFG_IMU_SAMPLE_HZ          250     /* High-rate MPU6050 FIFO/ISR stream (200-500 Hz) */
#define CFG_IMU_DT_SEC             (1.0 / (double)CFG_IMU_SAMPLE_HZ)
#define CFG_TELEMETRY_HZ           5       /* Slow telemetry rate (temp, speed, GPS, upload) */
#define CFG_TELEMETRY_DT_SEC       (1.0 / (double)CFG_TELEMETRY_HZ)
#define CFG_IMU_SUBSTEPS_PER_TICK  (CFG_IMU_SAMPLE_HZ / CFG_TELEMETRY_HZ) /* 50 substeps */

/* ---- Data Structure Capacities (Static, Malloc-Free) --------------------- */
#define CFG_CQ_DEFAULT_CAPACITY    128     /* Ring buffer black-box packet capacity */
#define CFG_CQ_MAX_CAPACITY        256     /* Static upper bound for ring buffer pool */
#define CFG_LL_POOL_SIZE           64      /* Static node pool size for outbox linked list */
#define CFG_RELAY_LOG_CAPACITY     32      /* Ring buffer of recent relay actuations */

/* ---- Detector Window & Thresholds ---------------------------------------- */
#define CFG_DET_WINDOW             16      /* Sliding window length for O(1) stats */
#define CFG_GRAVITY_MPS2           9.81    /* Standard gravity removed after calibration */

/* Multi-signal thresholds (calibrated on linear acceleration + raw magnitude) */
#define CFG_TH_CRASH_ACCEL_MPS2    25.0    /* Sudden total impact magnitude (m/s^2) */
#define CFG_TH_CRASH_SPEED_DROP    8.0     /* Minimum speed drop (km/h) for fused collision */
#define CFG_TH_EXTREME_IMPACT_MPS2 38.0    /* Unconditional severe impact (e.g. stationary hit) */

#define CFG_TH_ROLLOVER_GYRO_DPS   80.0    /* Combined roll/pitch angular rate (deg/s) */
#define CFG_TH_ROLLOVER_ACCEL_MPS2 14.0    /* Coupled lateral/vertical acceleration */
#define CFG_ROLLOVER_PERSIST_MS    60.0    /* Rollover tilt must persist for >= N ms */

/* Hysteresis thresholds for Hard Brake & Overheat */
#define CFG_TH_BRAKE_ENTER_AX     -12.0    /* Enter hard-brake state below -12 m/s^2 */
#define CFG_TH_BRAKE_EXIT_AX       -6.0    /* Exit hard-brake state above -6 m/s^2 */
#define CFG_TH_BRAKE_MIN_MAG       10.0    /* Minimum total accel magnitude for hard brake */

#define CFG_TH_OVERHEAT_ENTER_C   110.0    /* Trigger overheat alert above 110 C */
#define CFG_TH_OVERHEAT_EXIT_C    102.0    /* Clear overheat condition below 102 C */

#define CFG_TH_THEFT_ACCEL_MPS2    3.5     /* Linear accel movement threshold while armed */
#define CFG_TH_SENSOR_MAX_ACCEL   150.0    /* Rail saturation / fault threshold (m/s^2) */

#define CFG_TH_ZSCORE_NOISE        3.5     /* Pothole / road jitter filter gate */
#define CFG_TH_ZSCORE_ALERT        5.0     /* Mild statistical anomaly gate */
#define CFG_ALERT_COOLDOWN_SEC     2.0     /* Suppress duplicate alerts within window */

/* ---- Safe Relay State Machine Parameters --------------------------------- */
#define CFG_SM_SUSPECTED_WINDOW_S  3.0     /* Seconds in SUSPECTED before CONFIRMED */
#define CFG_SM_SAFE_CUT_SPEED_KMH  3.0     /* Only cut engine when speed <= 3.0 km/h */
#define CFG_SM_WATCHDOG_TIMEOUT_S  2.0     /* Watchdog timeout before fail-safe reset */
#define CFG_DEFAULT_OVERRIDE_PIN   "1234"  /* Default operator PIN / RFID token */

/* ---- Outbox & Persistence ------------------------------------------------ */
#define CFG_OUTBOX_PATH            "data/outbox.txt"
#define CFG_TIMELINE_PATH          "data/timeline.txt"
#define CFG_PUB_MAX_BACKOFF_SEC    32.0
#define CFG_STORAGE_BATCH_INTERVAL 2.0     /* Seconds between batched LittleFS/disk flushes */

/* ---- ESP32 Hardware Pin Placeholders (Used by hal_esp32 backend only) ---- */
#define CFG_PIN_I2C_SDA            21      /* MPU6050 SDA */
#define CFG_PIN_I2C_SCL            22      /* MPU6050 SCL */
#define CFG_MPU6050_I2C_ADDR       0x68
#define CFG_PIN_RELAY_ENGINE       25      /* NC Relay: LOW = Engine Run, HIGH = Cut-off */
#define CFG_PIN_RELAY_LIGHTS       26      /* NO Relay: HIGH = Hazard/Headlights ON */
#define CFG_PIN_RELAY_SIREN        27      /* NO Relay: HIGH = Siren ON */
#define CFG_PIN_RELAY_LOCK         14      /* NO Relay: HIGH = Doors Locked, LOW = Unlocked */
#define CFG_PIN_LM35_ADC1          34      /* ADC1_CH6 (GPIO34) — safe when Wi-Fi/radio active */
#define CFG_PIN_BATTERY_ADC1       35      /* ADC1_CH7 (GPIO35) — voltage divider input */
#define CFG_PIN_SIM7670_RX         16      /* UART2 RX connected to SIM7670C TX */
#define CFG_PIN_SIM7670_TX         17      /* UART2 TX connected to SIM7670C RX */
#define CFG_PIN_LORA_SS            5       /* LoRa SPI CS */
#define CFG_PIN_LORA_RST           15      /* LoRa Reset */
#define CFG_PIN_LORA_DIO0          4       /* LoRa IRQ */

/* ---- Remote Server & Security Placeholders ------------------------------- */
#define CFG_CLOUD_ENDPOINT_URL     "https://todo-your-backend.example.com/api/v1/telemetry"
#define CFG_DEVICE_ID              "CAR-ESP32-042"
#define CFG_DEVICE_PSK_PLACEHOLDER "TODO_REPLACE_WITH_PROVISIONED_SECRET_KEY"
#define CFG_CMD_MAX_SKEW_SEC       30.0    /* Max allowed timestamp drift for signed commands */

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
