/**
 * simulator.h — Vehicle telemetry generator + injectable crash scenarios.
 *
 * Supports both 250 Hz high-rate IMU generation and 5 Hz slow telemetry,
 * deterministic PRNG seeding, and scripted auto-demo story mode.
 */
#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "config.h"
#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SCENARIO_NONE           = 0,
    SCENARIO_HARD_BRAKE     = 1,
    SCENARIO_COLLISION      = 2,
    SCENARIO_ROLLOVER       = 3,
    SCENARIO_OVERHEAT       = 4,
    SCENARIO_POTHOLE        = 5, /* noise — should be filtered by detector */
    SCENARIO_HIGHWAY_CRUISE = 6,
    SCENARIO_THEFT          = 7,
    SCENARIO_GPS_LOSS       = 8,
    SCENARIO_SENSOR_FAULT   = 9
} Scenario;

typedef struct {
    int      vehicle_id;
    double   t;              /* simulation clock (seconds) */
    double   dt;             /* slow telemetry step (0.2s = 5 Hz) */
    double   imu_dt;         /* fast IMU step (0.004s = 250 Hz)   */
    double   speed;
    double   lat, lon;
    double   engine_temp;
    double   battery;
    unsigned rng;
    int      gps_valid;
    int      sensor_fault;

    Scenario pending;        /* scheduled via CLI          */
    double   pending_at;
    Scenario active;         /* currently shaping packets  */
    int      active_ticks;   /* remaining slow ticks       */
    int      imu_substep;    /* 0 .. CFG_IMU_SUBSTEPS_PER_TICK-1 */

    int      auto_demo;      /* 1 = run scripted story     */
    int      demo_stage;     /* current stage in story     */
} Simulator;

void simulator_init(Simulator *s, int vehicle_id);
void simulator_set_seed(Simulator *s, unsigned seed);
void simulator_set_autodemo(Simulator *s, int enabled);
void simulator_schedule(Simulator *s, Scenario sc, double at_time);
void simulator_inject(Simulator *s, Scenario sc); /* fire ASAP */

/**
 * Produce one high-rate (250 Hz) IMU-updated packet while keeping slow
 * channels (temp, GPS, battery) updated at 5 Hz.
 * Complexity: O(1).
 */
void simulator_step_fast(Simulator *s, TelemetryPacket *out);

/** Produce next 5 Hz TelemetryPacket (backward-compatible wrapper). O(1). */
void simulator_step(Simulator *s, TelemetryPacket *out);

Scenario scenario_from_name(const char *name);
const char *scenario_name(Scenario sc);

/* Bridge to hal_sim.c */
void hal_sim_set_clock(double t);
void hal_sim_feed_sensors(double ax, double ay, double az,
                          double gx, double gy, double gz,
                          double speed_kmh, double temp_c, double battery_pct,
                          double lat, double lon, int gps_valid, int imu_fault);

#ifdef __cplusplus
}
#endif

#endif /* SIMULATOR_H */
