/**
 * test_detector.c — noise ignored, real crashes caught, multi-signal fusion,
 * rollover persistence, hysteresis, cooldown, and IMU calibration verified.
 */
#include "../src/detector.h"
#include "../src/hal.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static TelemetryPacket base_pkt(double t)
{
    TelemetryPacket p;
    memset(&p, 0, sizeof(p));
    p.timestamp = t;
    p.vehicle_id = 7;
    p.speed = 60.0;
    p.accel_x = 0.1;
    p.accel_y = 0.0;
    p.accel_z = CFG_GRAVITY_MPS2;
    p.latitude = 28.6;
    p.longitude = 77.2;
    p.engine_temp = 90.0;
    p.battery = 80.0;
    p.gps_valid = 1;
    return p;
}

static void warm_baseline(Detector *d)
{
    int i;
    Alert ignore;
    /* Feed quiet packets so mean/variance settle (fill DET_WINDOW). */
    for (i = 0; i < DET_WINDOW + 4; i++) {
        TelemetryPacket p = base_pkt((double)i * CFG_IMU_DT_SEC);
        p.accel_x = (i % 2 == 0) ? 0.2 : -0.1;
        p.accel_z = 9.8 + ((i % 3) * 0.05);
        detector_process(d, &p, &ignore);
    }
}

static void test_imu_boot_calibration_and_gravity_removal(void)
{
    HalImuCalibration cal;
    HalImuSample out;

    memset(&cal, 0, sizeof(cal));
    cal.remove_gravity = 1;
    hal_imu_set_mounting(&cal, 0, 1, 2, 1, 1, 1);
    assert(hal_imu_calibrate(&cal, 64) == 0);
    assert(cal.calibrated == 1);

    hal_imu_apply_calibration(&cal,
                              cal.accel_bias[0],
                              cal.accel_bias[1],
                              CFG_GRAVITY_MPS2 + cal.accel_bias[2],
                              cal.gyro_bias[0],
                              cal.gyro_bias[1],
                              cal.gyro_bias[2],
                              &out);

    assert(fabs(out.ax) < 1e-4);
    assert(fabs(out.ay) < 1e-4);
    assert(fabs(out.az - CFG_GRAVITY_MPS2) < 1e-4);
    assert(fabs(out.lin_az) < 1e-4);
    assert(fabs(out.gx) < 1e-4);

    hal_imu_set_mounting(&cal, 1, 0, 2, -1, 1, -1);
    hal_imu_apply_calibration(&cal,
                              1.0 + cal.accel_bias[0],
                              5.0 + cal.accel_bias[1],
                              CFG_GRAVITY_MPS2 + cal.accel_bias[2],
                              0.0, 0.0, 0.0,
                              &out);
    assert(fabs(out.ax - (-5.0)) < 1e-3);
    assert(fabs(out.ay - 1.0) < 1e-3);
    assert(fabs(out.az - (-CFG_GRAVITY_MPS2)) < 1e-3);

    printf("  OK boot-time IMU calibration, mounting orientation & gravity removal\n");
}

static void test_noise_ignored(void)
{
    Detector d;
    Alert a;
    TelemetryPacket p;
    AlertType t;

    detector_init(&d);
    warm_baseline(&d);

    /* Single-sample pothole spike without speed drop -> strictly ALERT_NONE */
    p = base_pkt(100.0);
    p.accel_z = 16.0;
    p.accel_y = 4.0;
    t = detector_process(&d, &p, &a);
    assert(t == ALERT_NONE);
    printf("  OK single-sample pothole filtered as %s (%s)\n",
           alert_type_name(t), d.last_reason);
}

static void test_collision_fusion_and_cooldown(void)
{
    Detector d;
    Alert a;
    TelemetryPacket p;
    AlertType t;

    detector_init(&d);
    warm_baseline(&d);

    /* Moderate-high |a| (~28 m/s^2) WITHOUT speed drop should NOT trigger collision */
    p = base_pkt(150.0);
    p.speed   = 60.0; /* no speed drop */
    p.accel_x = -8.0;
    p.accel_y = 10.0;
    p.accel_z = 25.0;
    t = detector_process(&d, &p, &a);
    assert(t == ALERT_NONE);

    /* High |a| WITH sudden speed drop (60 -> 10 km/h) -> ALERT_COLLISION */
    p = base_pkt(200.0);
    p.speed   = 10.0;
    p.accel_x = -26.0;
    p.accel_y = 12.0;
    p.accel_z = 18.0;
    p.latitude = 28.612345;
    p.longitude = 77.209876;
    t = detector_process(&d, &p, &a);
    assert(t == ALERT_COLLISION);
    assert(a.severity == SEV_CRITICAL);
    assert(a.lat == p.latitude);
    assert(a.lon == p.longitude);
    assert(a.timestamp == p.timestamp);

    /* Immediate follow-up sample within cooldown window -> suppressed */
    p.timestamp = 200.5;
    t = detector_process(&d, &p, &a);
    assert(t == ALERT_NONE);

    printf("  OK multi-signal collision fusion + localization + cooldown\n");
}

static void test_hard_brake_and_overheat_hysteresis(void)
{
    Detector d;
    Alert a;
    TelemetryPacket p;
    AlertType t;

    detector_init(&d);
    warm_baseline(&d);

    /* Enter hard brake (ax = -15 < -12) */
    p = base_pkt(300.0);
    p.accel_x = -15.0;
    p.accel_z = 10.0;
    t = detector_process(&d, &p, &a);
    assert(t == ALERT_HARD_BRAKE);

    /* After cooldown expires, still braking at ax = -13 -> hysteresis suppresses re-fire */
    p = base_pkt(305.0);
    p.accel_x = -13.0;
    p.accel_z = 10.0;
    t = detector_process(&d, &p, &a);
    assert(t == ALERT_NONE);

    /* Release brake (ax = -2 > -6 exit threshold), then brake hard again -> fires */
    p = base_pkt(306.0);
    p.accel_x = -2.0;
    detector_process(&d, &p, &a);

    p = base_pkt(310.0);
    p.accel_x = -15.0;
    p.accel_z = 10.0;
    t = detector_process(&d, &p, &a);
    assert(t == ALERT_HARD_BRAKE);
    printf("  OK hard brake with hysteresis\n");

    /* Overheat hysteresis test */
    detector_init(&d);
    warm_baseline(&d);
    p = base_pkt(400.0);
    p.engine_temp = 115.0;
    t = detector_process(&d, &p, &a);
    assert(t == ALERT_OVERHEAT);

    /* Still at 105 C (> 102 C exit) after cooldown -> latched, no re-trigger */
    p = base_pkt(405.0);
    p.engine_temp = 112.0;
    t = detector_process(&d, &p, &a);
    assert(t == ALERT_NONE);

    /* Cool down below 102 C, then overheat again -> fires */
    p = base_pkt(406.0);
    p.engine_temp = 98.0;
    detector_process(&d, &p, &a);

    p = base_pkt(410.0);
    p.engine_temp = 116.0;
    t = detector_process(&d, &p, &a);
    assert(t == ALERT_OVERHEAT);
    printf("  OK overheat with hysteresis\n");
}

static void test_rollover_persistence(void)
{
    Detector d;
    Alert a;
    TelemetryPacket p;
    AlertType t = ALERT_NONE;
    int i;

    detector_init(&d);
    warm_baseline(&d);

    /* Brief 2-sample gyro glitch (8 ms < 60 ms) -> must NOT trigger rollover */
    for (i = 0; i < 2; i++) {
        p = base_pkt(500.0 + i * CFG_IMU_DT_SEC);
        p.gyro_x = 110.0;
        p.gyro_y = 95.0;
        p.accel_z = 15.0;
        t = detector_process(&d, &p, &a);
        assert(t == ALERT_NONE);
    }

    /* Reset with 1 quiet sample */
    p = base_pkt(501.0);
    detector_process(&d, &p, &a);

    /* Sustained rollover (>= 15 samples = 60 ms at 250 Hz) -> triggers ROLLOVER */
    for (i = 0; i < 20; i++) {
        p = base_pkt(510.0 + i * CFG_IMU_DT_SEC);
        p.gyro_x = 110.0;
        p.gyro_y = 95.0;
        p.accel_z = 18.0;
        p.accel_x = 10.0;
        t = detector_process(&d, &p, &a);
        if (t == ALERT_ROLLOVER)
            break;
    }
    assert(t == ALERT_ROLLOVER);
    printf("  OK rollover persistence (>= %.0f ms)\n", CFG_ROLLOVER_PERSIST_MS);
}

int main(void)
{
    printf("test_detector\n");
    test_imu_boot_calibration_and_gravity_removal();
    test_noise_ignored();
    test_collision_fusion_and_cooldown();
    test_hard_brake_and_overheat_hysteresis();
    test_rollover_persistence();
    printf("ALL PASSED\n");
    return 0;
}
