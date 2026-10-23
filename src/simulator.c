/**
 * simulator.c — Synthetic driving + noise + injectable anomalies.
 *
 * Generates normal cruise, occasional pothole spikes (noise), and CLI/keyboard
 * scenarios: highway cruise, hard brake, collision, rollover, overheating,
 * pothole, theft attempt, GPS loss, sensor fault.
 *
 * Supports 250 Hz high-rate IMU sub-stepping with HAL calibration + 5 Hz slow
 * telemetry updates.
 *
 * TIME: O(1) per step.
 */
#include "simulator.h"
#include "hal.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned xs(unsigned *st)
{
    unsigned x = *st ? *st : 1u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *st = x;
    return x;
}

static double frand(Simulator *s, double lo, double hi)
{
    double u = (xs(&s->rng) & 0xFFFFFF) / (double)0x1000000;
    return lo + (hi - lo) * u;
}

static int ticks_for(Scenario sc)
{
    switch (sc) {
    case SCENARIO_HIGHWAY_CRUISE: return 15;
    case SCENARIO_OVERHEAT:       return 20;
    case SCENARIO_COLLISION:      return 2;
    case SCENARIO_ROLLOVER:       return 4;
    case SCENARIO_HARD_BRAKE:     return 3;
    case SCENARIO_POTHOLE:        return 1;
    case SCENARIO_THEFT:          return 4;
    case SCENARIO_GPS_LOSS:       return 15;
    case SCENARIO_SENSOR_FAULT:   return 3;
    default:                      return 0;
    }
}

void simulator_init(Simulator *s, int vehicle_id)
{
    memset(s, 0, sizeof(*s));
    s->vehicle_id   = vehicle_id;
    s->t            = 0.0;
    s->dt           = CFG_TELEMETRY_DT_SEC; /* 0.2 s (5 Hz) */
    s->imu_dt       = CFG_IMU_DT_SEC;       /* 0.004 s (250 Hz) */
    s->speed        = 60.0;
    s->lat          = 28.6139;
    s->lon          = 77.2090;
    s->engine_temp  = 85.0;
    s->battery      = 78.0;
    s->rng          = 0xDEADBEEFu;
    s->gps_valid    = 1;
    s->sensor_fault = 0;
    s->pending      = SCENARIO_NONE;
    s->active       = SCENARIO_NONE;
    hal_imu_init();
}

void simulator_set_seed(Simulator *s, unsigned seed)
{
    if (!s)
        return;
    s->rng = seed ? seed : 1u;
}

void simulator_set_autodemo(Simulator *s, int enabled)
{
    if (!s)
        return;
    s->auto_demo  = enabled ? 1 : 0;
    s->demo_stage = 0;
}

void simulator_schedule(Simulator *s, Scenario sc, double at_time)
{
    s->pending = sc;
    s->pending_at = at_time;
    printf("[SIM] Scheduled %s at t=%.1f\n", scenario_name(sc), at_time);
}

void simulator_inject(Simulator *s, Scenario sc)
{
    s->active = sc;
    s->active_ticks = ticks_for(sc);
    printf("[SIM] Inject %s now (t=%.2f)\n", scenario_name(sc), s->t);
}

Scenario scenario_from_name(const char *name)
{
    if (!name) return SCENARIO_NONE;
    if (!strcmp(name, "cruise") || !strcmp(name, "highway"))    return SCENARIO_HIGHWAY_CRUISE;
    if (!strcmp(name, "hard_brake") || !strcmp(name, "brake"))  return SCENARIO_HARD_BRAKE;
    if (!strcmp(name, "collision") || !strcmp(name, "crash"))   return SCENARIO_COLLISION;
    if (!strcmp(name, "rollover"))                              return SCENARIO_ROLLOVER;
    if (!strcmp(name, "overheat") || !strcmp(name, "overheating")) return SCENARIO_OVERHEAT;
    if (!strcmp(name, "pothole") || !strcmp(name, "noise"))     return SCENARIO_POTHOLE;
    if (!strcmp(name, "theft"))                                 return SCENARIO_THEFT;
    if (!strcmp(name, "gps_loss") || !strcmp(name, "nogps"))    return SCENARIO_GPS_LOSS;
    if (!strcmp(name, "sensor_fault") || !strcmp(name, "fault")) return SCENARIO_SENSOR_FAULT;
    return SCENARIO_NONE;
}

const char *scenario_name(Scenario sc)
{
    switch (sc) {
    case SCENARIO_HIGHWAY_CRUISE: return "highway_cruise";
    case SCENARIO_HARD_BRAKE:     return "hard_brake";
    case SCENARIO_COLLISION:      return "collision";
    case SCENARIO_ROLLOVER:       return "rollover";
    case SCENARIO_OVERHEAT:       return "overheat";
    case SCENARIO_POTHOLE:        return "pothole";
    case SCENARIO_THEFT:          return "theft";
    case SCENARIO_GPS_LOSS:       return "gps_loss";
    case SCENARIO_SENSOR_FAULT:   return "sensor_fault";
    default:                      return "none";
    }
}

/* Auto-demo scripted story progression */
static void update_autodemo_story(Simulator *s)
{
    if (!s->auto_demo || s->active != SCENARIO_NONE)
        return;

    if (s->demo_stage == 0 && s->t >= 3.0) {
        s->demo_stage = 1;
        simulator_inject(s, SCENARIO_POTHOLE);
    } else if (s->demo_stage == 1 && s->t >= 8.0) {
        s->demo_stage = 2;
        simulator_inject(s, SCENARIO_HARD_BRAKE);
    } else if (s->demo_stage == 2 && s->t >= 15.0) {
        s->demo_stage = 3;
        simulator_inject(s, SCENARIO_OVERHEAT);
    } else if (s->demo_stage == 3 && s->t >= 25.0) {
        s->demo_stage = 4;
        simulator_inject(s, SCENARIO_COLLISION);
    } else if (s->demo_stage == 4 && s->t >= 36.0) {
        s->demo_stage = 5;
        simulator_inject(s, SCENARIO_ROLLOVER);
    }
}

static void shape_channels(Simulator *s, int is_substep_zero,
                           double *ax, double *ay, double *az,
                           double *gx, double *gy, double *gz)
{
    *ax = frand(s, -0.4, 0.4);
    *ay = frand(s, -0.3, 0.3);
    *az = CFG_GRAVITY_MPS2 + frand(s, -0.2, 0.2);
    *gx = frand(s, -1.0, 1.0);
    *gy = frand(s, -1.0, 1.0);
    *gz = frand(s, -1.5, 1.5);

    s->gps_valid    = 1;
    s->sensor_fault = 0;

    /* Rare natural pothole (single sample spike — detector should filter out). */
    if (s->active == SCENARIO_NONE && is_substep_zero && frand(s, 0.0, 1.0) < 0.02) {
        *az += frand(s, 4.0, 8.0);
        *ay += frand(s, -3.0, 3.0);
    }

    if (s->active != SCENARIO_NONE && s->active_ticks > 0) {
        switch (s->active) {
        case SCENARIO_HIGHWAY_CRUISE:
            s->speed = 105.0 + frand(s, -1.0, 1.0);
            s->engine_temp = 90.0;
            break;
        case SCENARIO_HARD_BRAKE:
            *ax = -18.0 + frand(s, -1.0, 1.0);
            if (is_substep_zero)
                s->speed *= 0.78;
            break;
        case SCENARIO_COLLISION:
            *ax = -30.0 + frand(s, -5.0, 5.0);
            *ay = frand(s, -20.0, 20.0);
            *az = 25.0 + frand(s, -5.0, 10.0);
            s->speed = 0.0;
            break;
        case SCENARIO_ROLLOVER:
            *gx = 120.0 + frand(s, -10.0, 10.0);
            *gy = 90.0 + frand(s, -10.0, 10.0);
            *az = 20.0;
            *ax = frand(s, -15.0, 15.0);
            if (is_substep_zero)
                s->speed *= 0.5;
            break;
        case SCENARIO_OVERHEAT:
            s->engine_temp = 115.0 + frand(s, 0.0, 8.0);
            break;
        case SCENARIO_POTHOLE:
            /* Single-substep road bump — transient vertical spike */
            if (s->imu_substep == 0) {
                *az = 14.0 + frand(s, -1.0, 2.0);
                *ay = frand(s, -4.0, 4.0);
            }
            break;
        case SCENARIO_THEFT:
            s->speed = 5.0 + frand(s, 0.0, 5.0);
            *ax = 5.5 + frand(s, -0.5, 1.0);
            *ay = 4.0 + frand(s, -1.0, 1.0);
            break;
        case SCENARIO_GPS_LOSS:
            s->gps_valid = 0;
            break;
        case SCENARIO_SENSOR_FAULT:
            *ax = 220.0; /* saturated rail */
            *ay = -220.0;
            s->sensor_fault = 1;
            break;
        default:
            break;
        }
    }
}

void simulator_step_fast(Simulator *s, TelemetryPacket *out)
{
    double ax, ay, az, gx, gy, gz;
    HalImuSample cal_imu;
    int is_slow_boundary;

    if (!s || !out)
        return;

    is_slow_boundary = (s->imu_substep == 0);

    if (is_slow_boundary) {
        update_autodemo_story(s);
        if (s->pending != SCENARIO_NONE && s->t >= s->pending_at &&
            s->active == SCENARIO_NONE) {
            s->active = s->pending;
            s->active_ticks = ticks_for(s->pending);
            s->pending = SCENARIO_NONE;
            printf("[SIM] Scenario %s FIRED at t=%.2f\n",
                   scenario_name(s->active), s->t);
        }
    }

    shape_channels(s, is_slow_boundary, &ax, &ay, &az, &gx, &gy, &gz);

    /* Feed raw physics into HAL and read back calibrated sample */
    hal_sim_set_clock(s->t);
    hal_sim_feed_sensors(ax, ay, az, gx, gy, gz,
                         s->speed, s->engine_temp, s->battery,
                         s->lat, s->lon, s->gps_valid, s->sensor_fault);
    hal_imu_read_sample(&cal_imu);

    out->timestamp    = s->t;
    out->vehicle_id   = s->vehicle_id;
    out->speed        = hal_speed_read_kmh();
    out->accel_x      = cal_imu.ax;
    out->accel_y      = cal_imu.ay;
    out->accel_z      = cal_imu.az;
    out->gyro_x       = cal_imu.gx;
    out->gyro_y       = cal_imu.gy;
    out->gyro_z       = cal_imu.gz;
    out->latitude     = s->lat;
    out->longitude    = s->lon;
    out->engine_temp  = hal_temp_read_celsius();
    out->battery      = hal_battery_read_pct();
    out->gps_valid    = s->gps_valid;
    out->sensor_fault = s->sensor_fault;

    s->imu_substep++;
    s->t += s->imu_dt;

    if (s->imu_substep >= CFG_IMU_SUBSTEPS_PER_TICK) {
        s->imu_substep = 0;
        if (s->active != SCENARIO_NONE && s->active_ticks > 0) {
            s->active_ticks--;
            if (s->active_ticks <= 0)
                s->active = SCENARIO_NONE;
        } else {
            /* Mild cruise drift at 5 Hz slow boundary */
            s->speed += frand(s, -0.5, 0.5);
            if (s->speed < 20.0) s->speed = 20.0;
            if (s->speed > 100.0) s->speed = 100.0;
            s->engine_temp += frand(s, -0.05, 0.08);
            if (s->engine_temp < 80.0) s->engine_temp = 80.0;
            if (s->engine_temp > 100.0) s->engine_temp = 100.0;
        }
        s->lat += (s->speed / 111000.0) * s->dt * 0.05;
        s->lon += (s->speed / 111000.0) * s->dt * 0.03;
        s->battery -= 0.001;
        if (s->battery < 10.0) s->battery = 10.0;
    }
}

void simulator_step(Simulator *s, TelemetryPacket *out)
{
    double ax, ay, az, gx, gy, gz;
    HalImuSample cal_imu;

    if (!s || !out)
        return;

    update_autodemo_story(s);

    /* Start scheduled scenario when wall-clock reaches pending_at. */
    if (s->pending != SCENARIO_NONE && s->t >= s->pending_at &&
        s->active == SCENARIO_NONE) {
        s->active = s->pending;
        s->active_ticks = ticks_for(s->pending);
        s->pending = SCENARIO_NONE;
        printf("[SIM] Scenario %s FIRED at t=%.2f\n",
               scenario_name(s->active), s->t);
    }

    s->imu_substep = 0;
    shape_channels(s, 1, &ax, &ay, &az, &gx, &gy, &gz);

    if (s->active != SCENARIO_NONE && s->active_ticks > 0) {
        s->active_ticks--;
        if (s->active_ticks <= 0)
            s->active = SCENARIO_NONE;
    }

    hal_sim_set_clock(s->t);
    hal_sim_feed_sensors(ax, ay, az, gx, gy, gz,
                         s->speed, s->engine_temp, s->battery,
                         s->lat, s->lon, s->gps_valid, s->sensor_fault);
    hal_imu_read_sample(&cal_imu);

    out->timestamp    = s->t;
    out->vehicle_id   = s->vehicle_id;
    out->speed        = hal_speed_read_kmh();
    out->accel_x      = cal_imu.ax;
    out->accel_y      = cal_imu.ay;
    out->accel_z      = cal_imu.az;
    out->gyro_x       = cal_imu.gx;
    out->gyro_y       = cal_imu.gy;
    out->gyro_z       = cal_imu.gz;
    out->latitude     = s->lat;
    out->longitude    = s->lon;
    out->engine_temp  = hal_temp_read_celsius();
    out->battery      = hal_battery_read_pct();
    out->gps_valid    = s->gps_valid;
    out->sensor_fault = s->sensor_fault;

    /* Mild cruise drift. */
    if (s->active == SCENARIO_NONE) {
        s->speed += frand(s, -0.5, 0.5);
        if (s->speed < 20.0) s->speed = 20.0;
        if (s->speed > 100.0) s->speed = 100.0;
        s->engine_temp += frand(s, -0.05, 0.08);
        if (s->engine_temp < 80.0) s->engine_temp = 80.0;
        if (s->engine_temp > 100.0 && s->active != SCENARIO_OVERHEAT)
            s->engine_temp = 100.0;
    }
    s->lat += (s->speed / 111000.0) * s->dt * 0.05;
    s->lon += (s->speed / 111000.0) * s->dt * 0.03;
    s->battery -= 0.001;
    if (s->battery < 10.0) s->battery = 10.0;

    s->t += s->dt;
}
