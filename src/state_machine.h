/**
 * state_machine.h — Safe Automotive Relay & Post-Crash State Machine.
 *
 * Replaces unsafe immediate engine cut-off with a multi-stage state machine:
 *   NORMAL -> SUSPECTED -> CONFIRMED -> LOCKDOWN
 *
 * Safety invariants:
 *   1. SUSPECTED activates ONLY Siren + Hazard Lights during a confirmation
 *      window (CFG_SM_SUSPECTED_WINDOW_S) so false triggers can be cancelled.
 *   2. CONFIRMED (crash/rollover) cuts the engine ONLY when vehicle speed is
 *      near zero (<= CFG_SM_SAFE_CUT_SPEED_KMH), preventing loss of power
 *      steering/braking while still moving, and UNLOCKS doors for rescue.
 *   3. LOCKDOWN (anti-theft / remote immobilize) locks doors, sounds alarm,
 *      and cuts the engine once speed is near zero.
 *   4. Operator PIN / RFID override cancels any active state back to NORMAL.
 *   5. Boot, brown-out, or watchdog expiry forces fail-safe relay state.
 *
 * Pure C11 — zero hardware #ifdefs. Uses hal_relay_* and hal_watchdog_*.
 */
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "config.h"
#include "hal.h"
#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SM_STATE_NORMAL    = 0,
    SM_STATE_SUSPECTED = 1,
    SM_STATE_CONFIRMED = 2,
    SM_STATE_LOCKDOWN  = 3
} VehicleState;

typedef struct {
    VehicleState state;
    AlertType    trigger_alert;
    double       state_entered_at;
    double       confirm_deadline;
    int          engine_cut_pending; /* 1 = waiting for speed <= 3 km/h */
    int          theft_mode;         /* 1 = transition to LOCKDOWN      */
    char         override_pin[16];
    char         status_msg[96];
    int          override_count;
} StateMachine;

void sm_init(StateMachine *sm);
void sm_reset_failsafe(StateMachine *sm, const char *reason);

/** Notify state machine of a newly detected alert at timestamp `now`. */
void sm_on_alert(StateMachine *sm, AlertType type, double now);

/**
 * Periodic state machine tick. Evaluates confirmation timers, speed-safe
 * engine cut interlock, and hardware watchdog.
 */
void sm_tick(StateMachine *sm, double now, double current_speed_kmh);

/**
 * Attempt to cancel/disarm via PIN or RFID token.
 * Returns 1 on valid PIN (transitions to NORMAL), 0 on invalid PIN.
 */
int  sm_override_pin(StateMachine *sm, const char *pin, double now);

/** Force transition to LOCKDOWN (e.g., from authenticated remote command). */
void sm_enter_lockdown(StateMachine *sm, double now, double current_speed_kmh, const char *reason);

const char *sm_state_name(VehicleState s);

#ifdef __cplusplus
}
#endif

#endif /* STATE_MACHINE_H */
