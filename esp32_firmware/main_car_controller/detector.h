/**
 * detector.h — Edge filtering / TinyML-style anomaly detector (pure C).
 *
 * Uses a sliding moving-average + z-score on accel magnitude, plus a
 * multi-signal decision rule (threshold tree) for crash / rollover / heat.
 *
 * All work is O(1) per packet (fixed-size history window).
 * Zero hardware-specific #ifdefs.
 */
#ifndef DETECTOR_H
#define DETECTOR_H

#include "config.h"
#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DET_WINDOW CFG_DET_WINDOW  /* moving-average window — fixed -> O(1) updates */

typedef struct {
    double accel_hist[DET_WINDOW]; /* |a| history for mean/variance          */
    double speed_hist[DET_WINDOW]; /* speed history for collision delta-v    */
    int    hist_count;
    int    hist_idx;
    double mean;
    double m2;                     /* variance (sample variance of window)   */
    double last_z;                 /* latest z-score for UI visualization    */
    double last_accel_mag;         /* latest |a| (m/s^2)                     */
    double last_linear_accel_mag;  /* gravity-removed |a_lin| (m/s^2)        */
    double last_gyro_mag;          /* latest |g| (deg/s)                     */

    /* High-rate persistence & hysteresis state */
    double sample_dt_ms;           /* sample interval in ms (e.g. 4ms @250Hz)*/
    double rollover_accum_ms;      /* accumulated ms of sustained rollover   */
    int    pothole_spike_ticks;    /* consecutive ticks of moderate Z spike  */
    int    brake_active;           /* hysteresis latch for hard braking      */
    int    overheat_active;        /* hysteresis latch for engine overheat   */
    int    armed;                  /* 1 = vehicle parked & anti-theft armed  */

    int    next_alert_id;
    double cooldown_until;         /* suppress re-alerts until this time     */
    char   last_reason[96];        /* explainable decision / filter reason   */
} Detector;

void detector_init(Detector *d);
void detector_set_sample_rate(Detector *d, int hz);
void detector_set_armed(Detector *d, int armed);

/** Set the next alert id (e.g. after reloading a persisted outbox). */
void detector_set_next_id(Detector *d, int next_id);

/**
 * Analyse one packet. Returns ALERT_NONE or a concrete AlertType.
 * On detection, fills *out_alert (caller enqueues into outbox).
 * Complexity: O(1) — fixed window, no heap, no network.
 */
AlertType detector_process(Detector *d, const TelemetryPacket *pkt, Alert *out_alert);

const char *alert_type_name(AlertType t);
const char *severity_name(Severity s);

#ifdef __cplusplus
}
#endif

#endif /* DETECTOR_H */
