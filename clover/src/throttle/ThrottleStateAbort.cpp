#include "ThrottleStateAbort.h"

void ThrottleStateAbort::init() {
    // Controller handles actuation now
}

std::pair<ThrottleStateOutput, ThrottleAbortData> ThrottleStateAbort::tick(uint32_t current_time, uint32_t entry_time, float default_fuel, float default_lox) {
    ThrottleStateOutput out{};
    ThrottleAbortData abort_data{};

    // Drive valves to nominal safe positions
    out.power_on = true;
    out.has_fuel_pos = true;
    out.fuel_pos = default_fuel;
    out.has_lox_pos = true;
    out.lox_pos = default_lox;

    // Run for ~0.5s before allowing state transition back to idle
    if (current_time - entry_time > 500) {
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    } else {
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
    }

    return std::make_pair(out, abort_data);
}
