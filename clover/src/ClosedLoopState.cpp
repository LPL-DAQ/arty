#include "ClosedLoopState.h"
#include "LookupTable.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(Controller, LOG_LEVEL_INF);

// 1. Define your specific curve for the closed-loop throttle
// (Remember: X values must have equal increments for the O(1) math!)
static const Point throttle_curve[] = {
    {0.0f, 0.0f},
    {100.0f, 25.0f},
    {200.0f, 50.0f},
    {300.0f, 75.0f},
    {400.0f, 100.0f}
};

static const int curve_size = sizeof(throttle_curve) / sizeof(throttle_curve[0]);

// 2. Create the LookupTable instance
static LookupTable throttle_table(throttle_curve, curve_size);

void ClosedLoopState::init() {
    LOG_INF("Entering Closed Loop Throttle Mode");
}

ControllerOutput ClosedLoopState::tick(bool has_ptc, float ptc_pressure) {
    ControllerOutput out;

    if (!has_ptc) {
        LOG_ERR("Lost feedback sensor! Aborting closed loop.");
        out.next_state = SystemState_STATE_ABORT;
        return out;
    }

    // 3. Calculate target throttle via the new LookupTable class
    float target_valve_pos = throttle_table.interpolate(ptc_pressure);

    out.set_fuel = true;
    out.fuel_pos = target_valve_pos;

    out.set_lox = true;
    out.lox_pos = target_valve_pos;

    out.next_state = SystemState_STATE_CLOSED_LOOP_THROTTLE;

    return out;
}