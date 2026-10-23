/**
 * detector.c — On-device multi-signal anomaly filter (no network calls, zero hardware #ifdefs).
 *
 * ALGORITHM (Phase 2 Multi-Signal Fusion):
 *   1. Compute accel magnitude |a|, gravity-removed linear accel |a_lin|, and |gyro|.
 *   2. Maintain fixed-size moving window (DET_WINDOW = 16) of |a| and speed -> O(1).
 *   3. Multi-signal fusion & hysteresis:
 *      - SENSOR_FAULT: NaN or rail saturation (|a| > 150 m/s^2)
 *      - ROLLOVER:     |gyro| > 80 deg/s AND |a| > 14 m/s^2 persisting >= 60 ms
 *      - COLLISION:    (|a| > 25 m/s^2 AND speed_drop >= 8 km/h) OR |a| > 38 m/s^2
 *      - HARD_BRAKE:   Hysteresis (enter ax < -12 m/s^2, exit ax > -6 m/s^2)
 *      - OVERHEAT:     Hysteresis (enter temp > 110 C, exit temp < 102 C)
 *      - THEFT:        Vehicle armed AND |a_lin| > 3.5 m/s^2
 *      - POTHOLE:      Single-sample road spike without speed drop -> FILTERED (NONE)
 *
 * TIME: O(1) per reading. SPACE: O(DET_WINDOW).
 */
#include "detector.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

void detector_init(Detector *d)
{
    if (!d)
        return;
    memset(d, 0, sizeof(*d));
    d->next_alert_id = 1;
    d->sample_dt_ms  = 1000.0 / (double)CFG_IMU_SAMPLE_HZ;
    snprintf(d->last_reason, sizeof(d->last_reason), "NOMINAL: baseline window warming");
}

void detector_set_sample_rate(Detector *d, int hz)
{
    if (!d || hz <= 0)
        return;
    d->sample_dt_ms = 1000.0 / (double)hz;
}

void detector_set_armed(Detector *d, int armed)
{
    if (!d)
        return;
    d->armed = armed ? 1 : 0;
}

void detector_set_next_id(Detector *d, int next_id)
{
    if (d && next_id > 0)
        d->next_alert_id = next_id;
}

static double accel_mag(const TelemetryPacket *p)
{
    return sqrt(p->accel_x * p->accel_x +
                p->accel_y * p->accel_y +
                p->accel_z * p->accel_z);
}

static double linear_accel_mag(const TelemetryPacket *p)
{
    double laz = p->accel_z - CFG_GRAVITY_MPS2;
    return sqrt(p->accel_x * p->accel_x +
                p->accel_y * p->accel_y +
                laz * laz);
}

static double gyro_mag(const TelemetryPacket *p)
{
    return sqrt(p->gyro_x * p->gyro_x +
                p->gyro_y * p->gyro_y +
                p->gyro_z * p->gyro_z);
}

/* Compute max speed drop across the 16-sample sliding window — O(1). */
static double window_speed_drop(const Detector *d, double current_speed)
{
    int i;
    double max_prev = current_speed;
    for (i = 0; i < d->hist_count; i++) {
        if (d->speed_hist[i] > max_prev)
            max_prev = d->speed_hist[i];
    }
    return max_prev - current_speed;
}

/* Fixed-window update — O(1) since DET_WINDOW is a small constant (16). */
static void window_push(Detector *d, double x, double speed)
{
    d->accel_hist[d->hist_idx] = x;
    d->speed_hist[d->hist_idx] = speed;
    if (d->hist_count < DET_WINDOW)
        d->hist_count++;
    d->hist_idx = (d->hist_idx + 1) % DET_WINDOW;

    {
        int i, n = d->hist_count;
        double sum = 0.0, var = 0.0;
        for (i = 0; i < n; i++)
            sum += d->accel_hist[i];
        d->mean = sum / (double)n;
        for (i = 0; i < n; i++) {
            double diff = d->accel_hist[i] - d->mean;
            var += diff * diff;
        }
        d->m2 = (n > 1) ? var / (double)(n - 1) : 0.0;
    }
}

static Severity severity_for(AlertType t)
{
    switch (t) {
    case ALERT_COLLISION:    return SEV_CRITICAL;
    case ALERT_ROLLOVER:     return SEV_CRITICAL;
    case ALERT_THEFT:        return SEV_HIGH;
    case ALERT_HARD_BRAKE:   return SEV_HIGH;
    case ALERT_OVERHEAT:     return SEV_HIGH;
    case ALERT_SENSOR_FAULT: return SEV_MEDIUM;
    case ALERT_ANOMALY:      return SEV_MEDIUM;
    default:                 return SEV_LOW;
    }
}

static void fill_alert(Detector *d, Alert *out, AlertType type,
                       const TelemetryPacket *pkt)
{
    out->id = d->next_alert_id++;
    out->type = type;
    out->severity = severity_for(type);
    out->timestamp = pkt->timestamp;
    out->lat = pkt->latitude;
    out->lon = pkt->longitude;
    out->retry_count = 0;
    out->next_retry_time = pkt->timestamp; /* publish ASAP */
}

AlertType detector_process(Detector *d, const TelemetryPacket *pkt, Alert *out_alert)
{
    double am, lam, gm, z, stddev, speed_drop;
    AlertType type = ALERT_NONE;

    if (!d || !pkt)
        return ALERT_NONE;

    am  = accel_mag(pkt);
    lam = linear_accel_mag(pkt);
    gm  = gyro_mag(pkt);

    d->last_accel_mag        = am;
    d->last_linear_accel_mag = lam;
    d->last_gyro_mag         = gm;

    /* 1. Sensor fault check before polluting sliding window */
    if (pkt->sensor_fault || isnan(am) || isnan(gm) || am > CFG_TH_SENSOR_MAX_ACCEL) {
        if (pkt->timestamp >= d->cooldown_until) {
            type = ALERT_SENSOR_FAULT;
            snprintf(d->last_reason, sizeof(d->last_reason),
                     "SENSOR_FAULT: rail saturation/fault |a|=%.1f m/s^2", am);
            if (out_alert) {
                fill_alert(d, out_alert, type, pkt);
                d->cooldown_until = pkt->timestamp + CFG_ALERT_COOLDOWN_SEC;
            }
            return type;
        }
        return ALERT_NONE;
    }

    speed_drop = window_speed_drop(d, pkt->speed);
    window_push(d, am, pkt->speed);

    /* Update hysteresis exit conditions even during cooldown */
    if (d->brake_active && pkt->accel_x > CFG_TH_BRAKE_EXIT_AX) {
        d->brake_active = 0;
    }
    if (d->overheat_active && pkt->engine_temp < CFG_TH_OVERHEAT_EXIT_C) {
        d->overheat_active = 0;
    }

    /* Update rollover persistence accumulator */
    if (gm > CFG_TH_ROLLOVER_GYRO_DPS && am > CFG_TH_ROLLOVER_ACCEL_MPS2) {
        d->rollover_accum_ms += d->sample_dt_ms;
    } else {
        d->rollover_accum_ms = 0.0;
    }

    stddev = sqrt(d->m2);
    if (stddev < 1e-6)
        stddev = 1e-6;
    z = (am - d->mean) / stddev;
    d->last_z = z;

    /* Still update stats during cooldown, but do not emit another alert. */
    if (pkt->timestamp < d->cooldown_until) {
        snprintf(d->last_reason, sizeof(d->last_reason),
                 "COOLDOWN: active until t=%.2fs", d->cooldown_until);
        return ALERT_NONE;
    }

    /*
     * 2. Multi-signal fused decision tree:
     *    - Rollover requires gyro + accel persisting >= CFG_ROLLOVER_PERSIST_MS
     *    - Collision fuses high |a| with speed drop (or extreme impact > 38 m/s^2)
     *    - Hard brake & Overheat use hysteresis latches
     *    - Potholes (transient Z/accel spikes with < 2 ticks and no speed drop) are filtered
     */
    if (d->rollover_accum_ms >= CFG_ROLLOVER_PERSIST_MS) {
        type = ALERT_ROLLOVER;
        d->rollover_accum_ms = 0.0;
        snprintf(d->last_reason, sizeof(d->last_reason),
                 "ROLLOVER: |g|=%.1f deg/s & |a|=%.1f persisted >= %.0fms",
                 gm, am, CFG_ROLLOVER_PERSIST_MS);
    } else if ((am > CFG_TH_CRASH_ACCEL_MPS2 && speed_drop >= CFG_TH_CRASH_SPEED_DROP) ||
               (am > CFG_TH_EXTREME_IMPACT_MPS2)) {
        type = ALERT_COLLISION;
        snprintf(d->last_reason, sizeof(d->last_reason),
                 "COLLISION: |a|=%.1f m/s^2 + dv=-%.1f km/h (z=%.1f)",
                 am, speed_drop, z);
    } else if (pkt->accel_x < CFG_TH_BRAKE_ENTER_AX && am > CFG_TH_BRAKE_MIN_MAG) {
        if (!d->brake_active) {
            d->brake_active = 1;
            type = ALERT_HARD_BRAKE;
            snprintf(d->last_reason, sizeof(d->last_reason),
                     "HARD_BRAKE: ax=%.1f m/s^2 < %.1f (hysteresis latched)",
                     pkt->accel_x, CFG_TH_BRAKE_ENTER_AX);
        } else {
            snprintf(d->last_reason, sizeof(d->last_reason),
                     "BRAKE_HYSTERESIS: latched until ax > %.1f", CFG_TH_BRAKE_EXIT_AX);
        }
    } else if (pkt->engine_temp > CFG_TH_OVERHEAT_ENTER_C) {
        if (!d->overheat_active) {
            d->overheat_active = 1;
            type = ALERT_OVERHEAT;
            snprintf(d->last_reason, sizeof(d->last_reason),
                     "OVERHEAT: temp=%.1f C > %.1f C (hysteresis latched)",
                     pkt->engine_temp, CFG_TH_OVERHEAT_ENTER_C);
        } else {
            snprintf(d->last_reason, sizeof(d->last_reason),
                     "OVERHEAT_HYSTERESIS: latched until temp < %.1f C",
                     CFG_TH_OVERHEAT_EXIT_C);
        }
    } else if (d->armed && lam > CFG_TH_THEFT_ACCEL_MPS2) {
        type = ALERT_THEFT;
        snprintf(d->last_reason, sizeof(d->last_reason),
                 "THEFT: armed motion |a_lin|=%.2f m/s^2 > %.1f",
                 lam, CFG_TH_THEFT_ACCEL_MPS2);
    } else if (d->hist_count >= DET_WINDOW && z > CFG_TH_ZSCORE_NOISE) {
        /*
         * Pothole / road-bump discriminator:
         * Single-sample road noise spikes z-score briefly without a speed drop.
         * Require >= 3 consecutive samples above TH_ZSCORE_ALERT to flag ANOMALY.
         */
        d->pothole_spike_ticks++;
        if (z > CFG_TH_ZSCORE_ALERT && d->pothole_spike_ticks >= 3) {
            type = ALERT_ANOMALY;
            d->pothole_spike_ticks = 0;
            snprintf(d->last_reason, sizeof(d->last_reason),
                     "ANOMALY: sustained z=%.2f > %.1f", z, CFG_TH_ZSCORE_ALERT);
        } else {
            type = ALERT_NONE;
            snprintf(d->last_reason, sizeof(d->last_reason),
                     "FILTERED_POTHOLE: single-spike |a|=%.1f (z=%.1f, dv=%.1f) rejected",
                     am, z, speed_drop);
        }
    } else {
        d->pothole_spike_ticks = 0;
        snprintf(d->last_reason, sizeof(d->last_reason),
                 "NOMINAL: |a|=%.2f m/s^2 mean=%.2f z=%.2f", am, d->mean, z);
    }

    if (type != ALERT_NONE && out_alert) {
        fill_alert(d, out_alert, type, pkt);
        d->cooldown_until = pkt->timestamp + CFG_ALERT_COOLDOWN_SEC;
    }

    return type;
}

const char *alert_type_name(AlertType t)
{
    switch (t) {
    case ALERT_HARD_BRAKE:   return "HARD_BRAKE";
    case ALERT_COLLISION:    return "COLLISION";
    case ALERT_ROLLOVER:     return "ROLLOVER";
    case ALERT_OVERHEAT:     return "OVERHEAT";
    case ALERT_ANOMALY:      return "ANOMALY";
    case ALERT_THEFT:        return "THEFT";
    case ALERT_SENSOR_FAULT: return "SENSOR_FAULT";
    default:                 return "NONE";
    }
}

const char *severity_name(Severity s)
{
    switch (s) {
    case SEV_LOW:      return "LOW";
    case SEV_MEDIUM:   return "MEDIUM";
    case SEV_HIGH:     return "HIGH";
    case SEV_CRITICAL: return "CRITICAL";
    default:           return "?";
    }
}
