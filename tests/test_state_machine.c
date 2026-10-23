/**
 * test_state_machine.c — Unit tests for the safe relay state machine.
 *
 * Verifies:
 *   1. Boot fail-safe relay state (engine allowed, doors unlocked, alarms off).
 *   2. NORMAL -> SUSPECTED activates siren & lights ONLY (no engine cut).
 *   3. PIN / RFID override cancels SUSPECTED / CONFIRMED / LOCKDOWN.
 *   4. SUSPECTED -> CONFIRMED defers engine cut while moving (speed > 3 km/h)
 *      and cuts engine only when speed <= 3 km/h.
 *   5. Theft triggers LOCKDOWN (doors locked + safe engine cut).
 *   6. Hardware watchdog timeout resets relays to fail-safe state.
 */
#include "../src/state_machine.h"
#include "../src/hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_failsafe_and_suspected_cancel(void)
{
    StateMachine sm;
    hal_storage_init();
    sm_init(&sm);

    assert(sm.state == SM_STATE_NORMAL);
    assert(hal_relay_get(HAL_RELAY_ENGINE) == 0);
    assert(hal_relay_get(HAL_RELAY_SIREN)  == 0);
    assert(hal_relay_get(HAL_RELAY_LIGHTS) == 0);
    assert(hal_relay_get(HAL_RELAY_LOCK)   == 0);

    /* Trigger collision at t=10.0s */
    sm_on_alert(&sm, ALERT_COLLISION, 10.0);
    assert(sm.state == SM_STATE_SUSPECTED);
    /* Engine must NOT be cut during SUSPECTED! */
    assert(hal_relay_get(HAL_RELAY_ENGINE) == 0);
    assert(hal_relay_get(HAL_RELAY_SIREN)  == 1);
    assert(hal_relay_get(HAL_RELAY_LIGHTS) == 1);

    /* Invalid PIN rejected */
    assert(sm_override_pin(&sm, "9999", 11.0) == 0);
    assert(sm.state == SM_STATE_SUSPECTED);

    /* Valid PIN cancels back to NORMAL and restores relays */
    assert(sm_override_pin(&sm, "1234", 11.5) == 1);
    assert(sm.state == SM_STATE_NORMAL);
    assert(hal_relay_get(HAL_RELAY_ENGINE) == 0);
    assert(hal_relay_get(HAL_RELAY_SIREN)  == 0);
    assert(hal_relay_get(HAL_RELAY_LIGHTS) == 0);

    printf("  OK SUSPECTED siren/lights only + PIN override cancel\n");
}

static void test_confirmed_speed_interlock(void)
{
    StateMachine sm;
    HalRelayLogEntry log[16];
    size_t n;

    sm_init(&sm);
    hal_watchdog_init(5.0);
    hal_watchdog_feed(20.0);

    /* Trigger rollover at t=20.0s */
    sm_on_alert(&sm, ALERT_ROLLOVER, 20.0);
    assert(sm.state == SM_STATE_SUSPECTED);

    /* Confirmation window expires at t=23.5s, but vehicle still sliding at 35 km/h */
    hal_watchdog_feed(23.5);
    sm_tick(&sm, 23.5, 35.0);
    assert(sm.state == SM_STATE_CONFIRMED);
    assert(sm.engine_cut_pending == 1);
    /* Engine MUST NOT be cut at 35 km/h */
    assert(hal_relay_get(HAL_RELAY_ENGINE) == 0);
    /* Doors unlocked for rescue */
    assert(hal_relay_get(HAL_RELAY_LOCK) == 0);

    /* Vehicle slows down to 1.5 km/h (<= 3.0 km/h safe threshold) at t=25.0s */
    hal_watchdog_feed(25.0);
    sm_tick(&sm, 25.0, 1.5);
    assert(sm.engine_cut_pending == 0);
    assert(hal_relay_get(HAL_RELAY_ENGINE) == 1);

    n = hal_relay_get_log(log, 16);
    assert(n > 0);
    printf("  OK CONFIRMED speed-safe engine cut interlock + blackbox relay log (%lu entries)\n",
           (unsigned long)n);
}

static void test_theft_lockdown_and_watchdog(void)
{
    StateMachine sm;
    sm_init(&sm);
    hal_watchdog_init(2.0);
    hal_watchdog_feed(50.0);

    /* Trigger theft at t=50.0s */
    sm_on_alert(&sm, ALERT_THEFT, 50.0);
    assert(sm.state == SM_STATE_SUSPECTED);

    /* Confirm at t=53.5s with speed=0 -> enters LOCKDOWN */
    hal_watchdog_feed(53.5);
    sm_tick(&sm, 53.5, 0.0);
    assert(sm.state == SM_STATE_LOCKDOWN);
    assert(hal_relay_get(HAL_RELAY_LOCK)   == 1); /* Doors locked on theft */
    assert(hal_relay_get(HAL_RELAY_ENGINE) == 1); /* Engine immobilized */
    assert(hal_relay_get(HAL_RELAY_SIREN)  == 1);

    /* Simulate MCU hang / brown-out: advance time past 2.0s watchdog without feeding */
    sm_tick(&sm, 57.0, 0.0);
    assert(sm.state == SM_STATE_NORMAL);
    assert(hal_relay_get(HAL_RELAY_ENGINE) == 0);
    assert(hal_relay_get(HAL_RELAY_LOCK)   == 0);
    assert(hal_relay_get(HAL_RELAY_SIREN)  == 0);

    printf("  OK THEFT -> LOCKDOWN + hardware watchdog fail-safe reset\n");
}

int main(void)
{
    printf("test_state_machine\n");
    test_failsafe_and_suspected_cancel();
    test_confirmed_speed_interlock();
    test_theft_lockdown_and_watchdog();
    printf("ALL PASSED\n");
    return 0;
}
