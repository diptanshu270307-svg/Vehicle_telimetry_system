/**
 * state_machine.c — Safe Automotive Relay & Post-Crash State Machine.
 *
 * Pure C11 implementation with zero hardware #ifdefs.
 */
#include "state_machine.h"
#include <stdio.h>
#include <string.h>

static void log_relay_to_blackbox(double now, const char *action, const char *detail)
{
    char line[160];
    int n = snprintf(line, sizeof(line),
                     "RELAY_LOG t=%.3f action=%s detail=%s [ENG=%d SIR=%d LGT=%d LCK=%d]\n",
                     now, action, detail ? detail : "",
                     hal_relay_get(HAL_RELAY_ENGINE),
                     hal_relay_get(HAL_RELAY_SIREN),
                     hal_relay_get(HAL_RELAY_LIGHTS),
                     hal_relay_get(HAL_RELAY_LOCK));
    if (n > 0)
        hal_storage_append_file(CFG_TIMELINE_PATH, line, (size_t)n);
}

void sm_init(StateMachine *sm)
{
    if (!sm)
        return;
    memset(sm, 0, sizeof(*sm));
    sm->state = SM_STATE_NORMAL;
    sm->trigger_alert = ALERT_NONE;
    snprintf(sm->override_pin, sizeof(sm->override_pin), "%s", CFG_DEFAULT_OVERRIDE_PIN);
    snprintf(sm->status_msg, sizeof(sm->status_msg), "NORMAL: all systems nominal");
    hal_relay_init_failsafe();
}

void sm_reset_failsafe(StateMachine *sm, const char *reason)
{
    if (!sm)
        return;
    sm->state = SM_STATE_NORMAL;
    sm->trigger_alert = ALERT_NONE;
    sm->engine_cut_pending = 0;
    sm->theft_mode = 0;
    hal_relay_set(HAL_RELAY_ENGINE, 0, reason ? reason : "failsafe_reset");
    hal_relay_set(HAL_RELAY_SIREN,  0, reason ? reason : "failsafe_reset");
    hal_relay_set(HAL_RELAY_LIGHTS, 0, reason ? reason : "failsafe_reset");
    hal_relay_set(HAL_RELAY_LOCK,   0, reason ? reason : "failsafe_reset");
    snprintf(sm->status_msg, sizeof(sm->status_msg),
             "FAILSAFE_RESET (%s): relays restored to safe state",
             reason ? reason : "reset");
    log_relay_to_blackbox(hal_time_now(), "FAILSAFE_RESET", reason);
}

void sm_on_alert(StateMachine *sm, AlertType type, double now)
{
    if (!sm || type == ALERT_NONE)
        return;

    /* Mild warnings (HARD_BRAKE, OVERHEAT, SENSOR_FAULT) flash hazards briefly
     * without entering full crash/theft confirmation unless severe. */
    if (type == ALERT_OVERHEAT || type == ALERT_HARD_BRAKE) {
        if (sm->state == SM_STATE_NORMAL) {
            hal_relay_set(HAL_RELAY_LIGHTS, 1, alert_type_name(type));
            snprintf(sm->status_msg, sizeof(sm->status_msg),
                     "WARNING (%s): hazard lights activated", alert_type_name(type));
            log_relay_to_blackbox(now, "WARN_HAZARDS", alert_type_name(type));
        }
        return;
    }

    if (type == ALERT_COLLISION || type == ALERT_ROLLOVER || type == ALERT_THEFT) {
        /* Enter SUSPECTED if currently in NORMAL */
        if (sm->state == SM_STATE_NORMAL) {
            sm->state            = SM_STATE_SUSPECTED;
            sm->trigger_alert    = type;
            sm->state_entered_at = now;
            sm->confirm_deadline = now + CFG_SM_SUSPECTED_WINDOW_S;
            sm->theft_mode       = (type == ALERT_THEFT) ? 1 : 0;
            sm->engine_cut_pending = 0;

            /* SUSPECTED: siren and hazards ONLY — never cut engine here */
            hal_relay_set(HAL_RELAY_ENGINE, 0, "suspected_keep_engine");
            hal_relay_set(HAL_RELAY_SIREN,  1, "suspected_siren_on");
            hal_relay_set(HAL_RELAY_LIGHTS, 1, "suspected_hazards_on");

            snprintf(sm->status_msg, sizeof(sm->status_msg),
                     "SUSPECTED (%s): siren+hazards on; confirming in %.1fs (PIN to cancel)",
                     alert_type_name(type), CFG_SM_SUSPECTED_WINDOW_S);
            log_relay_to_blackbox(now, "ENTER_SUSPECTED", alert_type_name(type));
        }
    }
}

void sm_enter_lockdown(StateMachine *sm, double now, double current_speed_kmh, const char *reason)
{
    if (!sm)
        return;
    sm->state = SM_STATE_LOCKDOWN;
    sm->state_entered_at = now;
    sm->theft_mode = 1;

    hal_relay_set(HAL_RELAY_SIREN,  1, "lockdown_siren");
    hal_relay_set(HAL_RELAY_LIGHTS, 1, "lockdown_hazards");
    hal_relay_set(HAL_RELAY_LOCK,   1, "lockdown_doors_locked");

    if (current_speed_kmh <= CFG_SM_SAFE_CUT_SPEED_KMH) {
        sm->engine_cut_pending = 0;
        hal_relay_set(HAL_RELAY_ENGINE, 1, "lockdown_safe_engine_cut");
        snprintf(sm->status_msg, sizeof(sm->status_msg),
                 "LOCKDOWN (%s): doors locked, engine cut (speed=%.1f km/h)",
                 reason ? reason : "theft", current_speed_kmh);
    } else {
        sm->engine_cut_pending = 1;
        hal_relay_set(HAL_RELAY_ENGINE, 0, "wait_for_zero_speed");
        snprintf(sm->status_msg, sizeof(sm->status_msg),
                 "LOCKDOWN (%s): waiting for speed %.1f <= %.1f km/h to cut engine",
                 reason ? reason : "theft", current_speed_kmh, CFG_SM_SAFE_CUT_SPEED_KMH);
    }
    log_relay_to_blackbox(now, "ENTER_LOCKDOWN", reason ? reason : "theft");
}

void sm_tick(StateMachine *sm, double now, double current_speed_kmh)
{
    if (!sm)
        return;

    /* Check hardware watchdog first */
    if (hal_watchdog_check(now)) {
        sm_reset_failsafe(sm, "watchdog_timeout");
        return;
    }

    if (sm->state == SM_STATE_SUSPECTED) {
        if (now >= sm->confirm_deadline) {
            if (sm->theft_mode) {
                sm_enter_lockdown(sm, now, current_speed_kmh, "theft_confirmed");
            } else {
                sm->state = SM_STATE_CONFIRMED;
                sm->state_entered_at = now;
                hal_relay_set(HAL_RELAY_SIREN,  1, "confirmed_siren");
                hal_relay_set(HAL_RELAY_LIGHTS, 1, "confirmed_hazards");
                /* Crash safety: unlock doors so first responders can open car */
                hal_relay_set(HAL_RELAY_LOCK,   0, "confirmed_crash_unlock");

                if (current_speed_kmh <= CFG_SM_SAFE_CUT_SPEED_KMH) {
                    sm->engine_cut_pending = 0;
                    hal_relay_set(HAL_RELAY_ENGINE, 1, "confirmed_safe_engine_cut");
                    snprintf(sm->status_msg, sizeof(sm->status_msg),
                             "CONFIRMED (%s): speed=%.1f km/h -> ENGINE CUT & DOORS UNLOCKED",
                             alert_type_name(sm->trigger_alert), current_speed_kmh);
                } else {
                    sm->engine_cut_pending = 1;
                    hal_relay_set(HAL_RELAY_ENGINE, 0, "interlock_wait_low_speed");
                    snprintf(sm->status_msg, sizeof(sm->status_msg),
                             "CONFIRMED (%s): speed=%.1f km/h > %.1f -> engine cut DEFERRED",
                             alert_type_name(sm->trigger_alert),
                             current_speed_kmh, CFG_SM_SAFE_CUT_SPEED_KMH);
                }
                log_relay_to_blackbox(now, "ENTER_CONFIRMED", alert_type_name(sm->trigger_alert));
            }
        } else {
            double rem = sm->confirm_deadline - now;
            snprintf(sm->status_msg, sizeof(sm->status_msg),
                     "SUSPECTED (%s): confirming in %.1fs (enter PIN to cancel)",
                     alert_type_name(sm->trigger_alert), rem > 0.0 ? rem : 0.0);
        }
    } else if (sm->state == SM_STATE_CONFIRMED || sm->state == SM_STATE_LOCKDOWN) {
        /* If engine cut was deferred due to moving vehicle, cut once speed near zero */
        if (sm->engine_cut_pending && current_speed_kmh <= CFG_SM_SAFE_CUT_SPEED_KMH) {
            sm->engine_cut_pending = 0;
            hal_relay_set(HAL_RELAY_ENGINE, 1, "deferred_safe_engine_cut");
            snprintf(sm->status_msg, sizeof(sm->status_msg),
                     "%s: vehicle stopped (%.1f km/h) -> ENGINE CUT engaged",
                     sm_state_name(sm->state), current_speed_kmh);
            log_relay_to_blackbox(now, "DEFERRED_ENGINE_CUT", sm_state_name(sm->state));
        }
    }
}

int sm_override_pin(StateMachine *sm, const char *pin, double now)
{
    if (!sm || !pin)
        return 0;

    if (strcmp(pin, sm->override_pin) != 0) {
        snprintf(sm->status_msg, sizeof(sm->status_msg),
                 "OVERRIDE DENIED: invalid PIN/RFID token");
        log_relay_to_blackbox(now, "OVERRIDE_DENIED", "bad_pin");
        return 0;
    }

    sm->override_count++;
    sm->state = SM_STATE_NORMAL;
    sm->trigger_alert = ALERT_NONE;
    sm->engine_cut_pending = 0;
    sm->theft_mode = 0;

    hal_relay_set(HAL_RELAY_ENGINE, 0, "pin_override_restore");
    hal_relay_set(HAL_RELAY_SIREN,  0, "pin_override_restore");
    hal_relay_set(HAL_RELAY_LIGHTS, 0, "pin_override_restore");
    hal_relay_set(HAL_RELAY_LOCK,   0, "pin_override_restore");

    snprintf(sm->status_msg, sizeof(sm->status_msg),
             "NORMAL: cancelled via valid PIN/RFID override (#%d)",
             sm->override_count);
    log_relay_to_blackbox(now, "OVERRIDE_OK", "pin_verified");
    return 1;
}

const char *sm_state_name(VehicleState s)
{
    switch (s) {
    case SM_STATE_NORMAL:    return "NORMAL";
    case SM_STATE_SUSPECTED: return "SUSPECTED";
    case SM_STATE_CONFIRMED: return "CONFIRMED";
    case SM_STATE_LOCKDOWN:  return "LOCKDOWN";
    default:                 return "UNKNOWN";
    }
}
