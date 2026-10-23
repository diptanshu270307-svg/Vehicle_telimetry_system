/**
 * structs.h — Core data types for the Telemetry & Anomaly Reconstruction Engine.
 *
 * DS COURSE NOTES:
 *   TelemetryPacket  — plain C struct (contiguous fields). Access is O(1).
 *   Alert            — alert record stored as nodes in the outbox linked list.
 *
 * No dynamic allocation inside these structs themselves; containers own lifetime.
 */
#ifndef STRUCTS_H
#define STRUCTS_H

#include "config.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Alert classification ------------------------------------------------ */

typedef enum {
    ALERT_NONE         = 0,
    ALERT_HARD_BRAKE   = 1,
    ALERT_COLLISION    = 2,
    ALERT_ROLLOVER     = 3,
    ALERT_OVERHEAT     = 4,
    ALERT_ANOMALY      = 5,
    ALERT_THEFT        = 6,
    ALERT_SENSOR_FAULT = 7
} AlertType;

/* Higher value = more urgent. Used by linked-list severity insert. */
typedef enum {
    SEV_LOW      = 1,
    SEV_MEDIUM   = 2,
    SEV_HIGH     = 3,
    SEV_CRITICAL = 4
} Severity;

/* ---- Telemetry packet (one reading) -------------------------------------- */
/**
 * TelemetryPacket — single sensor snapshot from the vehicle.
 * Stored in the circular-queue "black box". Size is fixed -> O(1) copy.
 */
typedef struct {
    double timestamp;   /* seconds since engine start               */
    int    vehicle_id;
    double speed;       /* km/h                                     */
    double accel_x;     /* longitudinal acceleration (m/s^2)        */
    double accel_y;     /* lateral                                  */
    double accel_z;     /* vertical (includes +9.81g when flat)     */
    double gyro_x;      /* roll rate  (deg/s)                       */
    double gyro_y;      /* pitch rate                               */
    double gyro_z;      /* yaw rate                                 */
    double latitude;
    double longitude;
    double engine_temp; /* deg C                                    */
    double battery;     /* % charge                                 */
    int    gps_valid;   /* 1 = valid fix, 0 = GPS lost              */
    int    sensor_fault;/* 1 = IMU/ADC fault flagged                */
} TelemetryPacket;

/* ---- Alert (pending publication) ----------------------------------------- */
/**
 * Alert — anomaly event waiting to be ACKed by the cloud publisher.
 * Lives inside a LinkedListNode; retry fields support exponential backoff.
 */
typedef struct {
    int       id;
    AlertType type;
    Severity  severity;
    double    timestamp;
    double    lat;
    double    lon;
    int       retry_count;
    double    next_retry_time; /* when we may attempt publish again  */
} Alert;

/* Human-readable helpers (defined in detector.c) */
const char *alert_type_name(AlertType t);
const char *severity_name(Severity s);

#ifdef __cplusplus
}
#endif

#endif /* STRUCTS_H */
