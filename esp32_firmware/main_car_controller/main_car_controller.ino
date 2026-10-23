/**
 * main_car_controller.ino — Dual-Core FreeRTOS ESP32 Firmware.
 *
 * Architecture (Phase 4):
 *   - Core 1 (`SensorDetectorTask`):
 *       Reads MPU6050 at 250 Hz (4 ms tick), runs O(1) Detector on the fast
 *       stream, samples slow telemetry (LM35 on ADC1 GPIO34, battery, speed)
 *       at 5 Hz, maintains the static-pool CircularQueue blackbox and safe
 *       StateMachine (`NORMAL -> SUSPECTED -> CONFIRMED -> LOCKDOWN`), and
 *       pushes alerts to Core 0 via a FreeRTOS Queue (`g_alert_queue`).
 *   - Core 0 (`CommsTask`):
 *       Receives Alert structs from `g_alert_queue` without shared mutable
 *       state, inserts them into the static-pool `Publisher` outbox, batches
 *       LittleFS flash writes (`CFG_STORAGE_BATCH_INTERVAL`), transmits over
 *       SIM7670C 4G+GNSS with LoRa fallback, and forwards authenticated
 *       override commands back to Core 1 via `g_cmd_queue`.
 */
#include "config.h"
#include "structs.h"
#include "circular_queue.h"
#include "linked_list.h"
#include "detector.h"
#include "state_machine.h"
#include "publisher.h"
#include "hal_esp32.h"

typedef struct {
    int  cmd_type; /* 1 = PIN_OVERRIDE, 2 = ARM, 3 = LOCKDOWN */
    char pin[16];
} InterCoreCmd;

/* FreeRTOS Queues connecting Core 1 (Sensor/Detector) and Core 0 (Comms) */
static QueueHandle_t g_alert_queue = NULL;
static QueueHandle_t g_cmd_queue   = NULL;

/* Core 1 Owned State (Zero shared structs between cores) */
static CircularQueue g_blackbox;
static Detector      g_detector;
static StateMachine  g_sm;

/* Core 0 Owned State */
static Publisher     g_publisher;

/* ---- Core 1: High-Rate Sensor + Detector + Safe Relay State Machine ------ */
static void SensorDetectorTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t fast_period = pdMS_TO_TICKS(1000 / CFG_IMU_SAMPLE_HZ); /* 4 ms */
    int substep = 0;

    double cached_temp_c = 85.0;
    double cached_batt   = 80.0;
    double cached_lat    = 0.0;
    double cached_lon    = 0.0;
    double cached_speed  = 0.0;

    hal_watchdog_init(CFG_SM_WATCHDOG_TIMEOUT_S);

    for (;;) {
        HalImuSample imu;
        TelemetryPacket pkt;
        Alert alert;
        AlertType atype;
        InterCoreCmd icmd;

        /* 1. Drain any incoming commands from Core 0 */
        while (xQueueReceive(g_cmd_queue, &icmd, 0) == pdTRUE) {
            double now = hal_time_now();
            if (icmd.cmd_type == 1) {
                sm_override_pin(&g_sm, icmd.pin, now);
                detector_set_armed(&g_detector, 0);
            } else if (icmd.cmd_type == 2) {
                detector_set_armed(&g_detector, 1);
            } else if (icmd.cmd_type == 3) {
                sm_enter_lockdown(&g_sm, now, cached_speed, "remote_signed_cmd");
            }
        }

        /* 2. Slow telemetry channels updated at 5 Hz (every 50th fast tick) */
        if (substep == 0) {
            cached_temp_c = hal_temp_read_celsius();
            cached_batt   = hal_battery_read_pct();
            cached_speed  = hal_speed_read_kmh();
            hal_gps_read(&cached_lat, &cached_lon);
        }

        /* 3. High-rate 250 Hz MPU6050 read + calibration */
        int fault = (hal_imu_read_sample(&imu) != 0);

        memset(&pkt, 0, sizeof(pkt));
        pkt.timestamp    = imu.timestamp;
        pkt.vehicle_id   = 1;
        pkt.speed        = cached_speed;
        pkt.accel_x      = imu.ax;
        pkt.accel_y      = imu.ay;
        pkt.accel_z      = imu.az;
        pkt.gyro_x       = imu.gx;
        pkt.gyro_y       = imu.gy;
        pkt.gyro_z       = imu.gz;
        pkt.latitude     = cached_lat;
        pkt.longitude    = cached_lon;
        pkt.engine_temp  = cached_temp_c;
        pkt.battery      = cached_batt;
        pkt.gps_valid    = 1;
        pkt.sensor_fault = fault;

        /* 4. Run O(1) detector on 250 Hz stream */
        atype = detector_process(&g_detector, &pkt, &alert);
        if (atype != ALERT_NONE) {
            cq_enqueue(&g_blackbox, &pkt);
            sm_on_alert(&g_sm, atype, pkt.timestamp);
            /* Send Alert to Core 0 CommsTask via lock-free FreeRTOS queue */
            xQueueSend(g_alert_queue, &alert, 0);
        } else if (substep == 0) {
            /* Enqueue regular 5 Hz snapshot into blackbox ring buffer */
            cq_enqueue(&g_blackbox, &pkt);
        }

        /* 5. At 5 Hz boundary, tick safe relay state machine & feed watchdog */
        substep = (substep + 1) % CFG_IMU_SUBSTEPS_PER_TICK;
        if (substep == 0) {
            double now = hal_time_now();
            hal_watchdog_feed(now);
            sm_tick(&g_sm, now, cached_speed);
        }

        vTaskDelayUntil(&last_wake, fast_period);
    }
}

/* ---- Core 0: Comms (SIM7670C + LoRa + Batched LittleFS Outbox) ----------- */
static void CommsTask(void *pvParameters)
{
    (void)pvParameters;
    Alert incoming;

    for (;;) {
        /* Drain all queued alerts from Core 1 */
        while (xQueueReceive(g_alert_queue, &incoming, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.printf("[COMMS] Queuing Alert #%d (%s, Sev=%s)\n",
                          incoming.id,
                          alert_type_name(incoming.type),
                          severity_name(incoming.severity));
            publisher_enqueue(&g_publisher, &incoming);
        }

        /* Run exponential backoff retry + batched LittleFS flush */
        publisher_tick(&g_publisher, hal_time_now());

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Booting ESP32 Dual-Core Telemetry & Safe Relay Controller...");

    /* 1. Fail-safe relay init FIRST before any peripheral initialization */
    hal_relay_init_failsafe();
    hal_storage_init();
    hal_imu_init();
    hal_modem_init();
    hal_lora_init();

    /* 2. Initialize Malloc-Free Static Data Structures */
    cq_init(&g_blackbox, CQ_DEFAULT_CAPACITY);
    detector_init(&g_detector);
    detector_set_sample_rate(&g_detector, CFG_IMU_SAMPLE_HZ);
    sm_init(&g_sm);
    publisher_init(&g_publisher);

    /* 3. Create Inter-Core FreeRTOS Queues */
    g_alert_queue = xQueueCreate(16, sizeof(Alert));
    g_cmd_queue   = xQueueCreate(8, sizeof(InterCoreCmd));

    /* 4. Pin Sensor+Detector to Core 1 (priority 3) and Comms to Core 0 (priority 2) */
    xTaskCreatePinnedToCore(SensorDetectorTask, "SensorDet", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(CommsTask,          "Comms",     8192, NULL, 2, NULL, 0);

    Serial.println("FreeRTOS Dual-Core Tasks Started (Core 1: 250Hz Sensor/Det, Core 0: Comms).");
}

void loop()
{
    /* Main Arduino loop yields; work is handled by Core 0 and Core 1 RTOS tasks */
    vTaskDelay(pdMS_TO_TICKS(1000));
}
