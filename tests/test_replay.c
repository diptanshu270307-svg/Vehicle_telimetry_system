/**
 * test_replay.c — Recorded/Synthetic CSV Trace Replay Harness & Confusion Matrix.
 *
 * Generates synthetic 250 Hz CSV traces (drive, pothole, hard_brake, drop_test,
 * rollover, overheat, theft) in data/traces/ if not present, feeds them through
 * the detector, and prints Precision, Recall, F1, and a full Confusion Matrix.
 */
#include "../src/config.h"
#include "../src/detector.h"
#include "../src/hal.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  include <direct.h>
#else
#  include <sys/stat.h>
#endif

#define NUM_CLASSES 6
/* Class indices for confusion matrix:
 * 0 = NONE (Normal drive / Pothole filtered)
 * 1 = HARD_BRAKE
 * 2 = COLLISION (Drop test / Crash)
 * 3 = ROLLOVER
 * 4 = OVERHEAT
 * 5 = THEFT
 */
static const char *CLASS_NAMES[NUM_CLASSES] = {
    "NONE      ",
    "HARD_BRAKE",
    "COLLISION ",
    "ROLLOVER  ",
    "OVERHEAT  ",
    "THEFT     "
};

static int alert_to_class(AlertType t)
{
    switch (t) {
    case ALERT_HARD_BRAKE: return 1;
    case ALERT_COLLISION:  return 2;
    case ALERT_ROLLOVER:   return 3;
    case ALERT_OVERHEAT:   return 4;
    case ALERT_THEFT:      return 5;
    default:               return 0;
    }
}

static void ensure_trace_dir(void)
{
#ifdef _WIN32
    _mkdir("data");
    _mkdir("data/traces");
#else
    mkdir("data", 0755);
    mkdir("data/traces", 0755);
#endif
}

/* Generate a CSV trace file with 60 samples at 250 Hz (dt = 0.004s) */
static void generate_csv_trace(const char *path, const char *scenario, int variant)
{
    FILE *fp = fopen(path, "w");
    int i;
    double v_jitter = (variant - 2) * 0.25;

    if (!fp)
        return;

    fprintf(fp, "# timestamp,speed,ax,ay,az,gx,gy,gz,temp,armed\n");
    for (i = 0; i < 60; i++) {
        double t = i * CFG_IMU_DT_SEC;
        double spd = 65.0;
        double ax = ((i % 2 == 0) ? 0.15 : -0.15) + v_jitter * 0.1;
        double ay = ((i % 3 == 0) ? 0.10 : -0.10);
        double az = CFG_GRAVITY_MPS2 + ((i % 2 == 0) ? 0.05 : -0.05);
        double gx = 0.2, gy = -0.1, gz = 0.1;
        double temp = 88.0;
        int armed = 0;

        /* First 25 samples warm up the 16-sample baseline window */
        if (i >= 25 && i < 45) {
            if (!strcmp(scenario, "pothole")) {
                /* Single-sample road bump at i == 30 (no speed drop) */
                if (i == 30) {
                    az = 15.5 + v_jitter;
                    ay = 3.2;
                }
            } else if (!strcmp(scenario, "hard_brake")) {
                ax = -15.5 + v_jitter;
                az = 10.2;
                spd = 65.0 - (i - 24) * 1.2;
            } else if (!strcmp(scenario, "drop_test")) {
                if (i >= 28 && i <= 32) {
                    ax = -28.0 + v_jitter;
                    ay = 14.0;
                    az = 22.0;
                    spd = 12.0; /* sudden speed drop from 65 km/h */
                }
            } else if (!strcmp(scenario, "rollover")) {
                /* Persists for 20 samples = 80 ms (>= 60 ms requirement) */
                gx = 115.0 + v_jitter;
                gy = 92.0;
                ax = 9.0;
                az = 17.5;
                spd = 40.0;
            } else if (!strcmp(scenario, "overheat")) {
                temp = 116.0 + v_jitter;
            } else if (!strcmp(scenario, "theft")) {
                armed = 1;
                spd = 4.0;
                ax = 4.8 + v_jitter;
                ay = 3.5;
            }
        } else if (!strcmp(scenario, "theft")) {
            armed = 1;
            spd = 0.0;
        }

        fprintf(fp, "%.4f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%d\n",
                t, spd, ax, ay, az, gx, gy, gz, temp, armed);
    }
    fclose(fp);
}

static AlertType replay_csv_file(const char *path)
{
    FILE *fp = fopen(path, "r");
    char line[256];
    Detector det;
    Alert alert;
    AlertType first_detected = ALERT_NONE;

    if (!fp)
        return ALERT_NONE;

    detector_init(&det);
    detector_set_sample_rate(&det, CFG_IMU_SAMPLE_HZ);

    while (fgets(line, sizeof(line), fp)) {
        TelemetryPacket p;
        double t, spd, ax, ay, az, gx, gy, gz, temp;
        int armed = 0;

        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
            continue;

        if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d",
                   &t, &spd, &ax, &ay, &az, &gx, &gy, &gz, &temp, &armed) == 10) {
            AlertType res;
            memset(&p, 0, sizeof(p));
            p.timestamp   = t;
            p.vehicle_id  = 1;
            p.speed       = spd;
            p.accel_x     = ax;
            p.accel_y     = ay;
            p.accel_z     = az;
            p.gyro_x      = gx;
            p.gyro_y      = gy;
            p.gyro_z      = gz;
            p.engine_temp = temp;
            p.gps_valid   = 1;

            detector_set_armed(&det, armed);
            res = detector_process(&det, &p, &alert);
            if (res != ALERT_NONE && first_detected == ALERT_NONE) {
                first_detected = res;
            }
        }
    }
    fclose(fp);
    return first_detected;
}

int main(void)
{
    const struct {
        const char *name;
        AlertType   expected;
    } suites[] = {
        { "drive",      ALERT_NONE       },
        { "pothole",    ALERT_NONE       },
        { "hard_brake", ALERT_HARD_BRAKE },
        { "drop_test",  ALERT_COLLISION  },
        { "rollover",   ALERT_ROLLOVER   },
        { "overheat",   ALERT_OVERHEAT   },
        { "theft",      ALERT_THEFT      }
    };
    int matrix[NUM_CLASSES][NUM_CLASSES];
    int tp = 0, fp = 0, tn = 0, fn = 0;
    size_t s;
    int v, r, c;

    memset(matrix, 0, sizeof(matrix));
    ensure_trace_dir();

    printf("test_replay (CSV trace replay harness @ %d Hz)\n", CFG_IMU_SAMPLE_HZ);

    for (s = 0; s < sizeof(suites) / sizeof(suites[0]); s++) {
        for (v = 0; v < 5; v++) {
            char path[128];
            AlertType pred;
            int exp_cls, pred_cls;

            snprintf(path, sizeof(path), "data/traces/%s_%d.csv", suites[s].name, v);
            generate_csv_trace(path, suites[s].name, v);
            /* Also write canonical trace file on v == 0 */
            if (v == 0) {
                char canonical[128];
                snprintf(canonical, sizeof(canonical), "data/traces/%s.csv", suites[s].name);
                generate_csv_trace(canonical, suites[s].name, 0);
            }

            pred = replay_csv_file(path);
            exp_cls  = alert_to_class(suites[s].expected);
            pred_cls = alert_to_class(pred);
            matrix[exp_cls][pred_cls]++;

            if (suites[s].expected == ALERT_NONE) {
                if (pred == ALERT_NONE) tn++;
                else fp++;
            } else {
                if (pred == suites[s].expected) tp++;
                else fn++;
            }

            assert(pred == suites[s].expected);
        }
        printf("  OK trace suite %-10s -> expected %-10s (5/5 passed)\n",
               suites[s].name, alert_type_name(suites[s].expected));
    }

    printf("\n  === DETECTOR CONFUSION MATRIX (Rows=Actual, Cols=Predicted) ===\n");
    printf("  Actual \\ Pred | NONE   BRAKE  CRASH  ROLL   HEAT   THEFT\n");
    printf("  --------------+-----------------------------------------\n");
    for (r = 0; r < NUM_CLASSES; r++) {
        printf("  %s    |", CLASS_NAMES[r]);
        for (c = 0; c < NUM_CLASSES; c++) {
            printf(" %4d  ", matrix[r][c]);
        }
        printf("\n");
    }

    {
        double precision = (tp + fp > 0) ? (double)tp / (double)(tp + fp) : 1.0;
        double recall    = (tp + fn > 0) ? (double)tp / (double)(tp + fn) : 1.0;
        double f1        = (precision + recall > 0.0)
                           ? (2.0 * precision * recall / (precision + recall))
                           : 0.0;
        printf("  ---------------------------------------------------------------\n");
        printf("  TP=%d  TN=%d  FP=%d  FN=%d  |  Precision=%.3f  Recall=%.3f  F1=%.3f\n\n",
               tp, tn, fp, fn, precision, recall, f1);
        assert(fp == 0 && fn == 0);
    }

    printf("ALL PASSED\n");
    return 0;
}
