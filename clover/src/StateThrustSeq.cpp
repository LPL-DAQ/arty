#include "StateThrustSeq.h"
#include "LookupTable.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(Controller, LOG_LEVEL_INF);

void StateThrustSeq::init() {
    LOG_INF("Entering Closed Loop Throttle Mode");
}

std::pair<ControllerOutput, ThrustSequenceData> StateThrustSeq::tick(bool has_ptc, float ptc_pressure) {
    ControllerOutput out{};
    ThrustSequenceData data{};

    if (!has_ptc) {
        LOG_ERR("Lost feedback sensor! Aborting closed loop.");
        out.next_state = SystemState_STATE_ABORT;
        return {out, data};
    }

    // Calculate target throttle via lookup table
    float target_valve_pos = interpolate_throttle(ptc_pressure);

    out.set_fuel = true;
    out.fuel_pos = target_valve_pos;

    out.set_lox = true;
    out.lox_pos = target_valve_pos;

    out.next_state = SystemState_STATE_THRUST_SEQ;

    return {out, data};
}
