#include "ClosedLoopState.h"
#include "Controller.h"
#include "ThrottleValve.h"
#include "LookupTable.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(Controller, LOG_LEVEL_INF);

void ClosedLoopState::init() {
    LOG_INF("Entering Closed Loop Throttle Mode");
}

void ClosedLoopState::run(const Sensors& sensors) {
    // Example implementation using ptc401 as feedback.
    // The team can change this to use whichever sensor they prefer later.
    if (!sensors.has_ptc401) {
        LOG_ERR("Lost feedback sensor! Aborting closed loop.");
        Controller::trigger_abort();
        return;
    }

    float current_pressure = sensors.ptc401;

    // Calculate target throttle via lookup table
    float target_valve_pos = interpolate_throttle(current_pressure);

    // Command the valves (Can adjust to just command one valve if needed)
    FuelValve::tick(target_valve_pos);
    LoxValve::tick(target_valve_pos);
}

void ClosedLoopState::end() {
    FuelValve::stop();
    LoxValve::stop();
    LOG_INF("Exited Closed Loop Throttle Mode");
}
