#include "SequenceState.h"

void SequenceState::init() {
    // Timer is reset in handle_start_sequence before entering
}

ControllerOutput SequenceState::tick(uint32_t current_time, uint32_t start_time, Trace& fuel_trace, Trace& lox_trace) {
    ControllerOutput out;
    out.next_state = SystemState_STATE_SEQUENCE; // Assume we stay in this state by default

    float dt = current_time - start_time;
    auto f_target = fuel_trace.sample(dt);
    auto l_target = lox_trace.sample(dt);

    // If sample fails (e.g. past end of trace or error), sequence is over
    if (!f_target || !l_target) {
        out.next_state = SystemState_STATE_IDLE;
        return out;
    }

    // Pass the raw float values back to the controller
    out.set_fuel = true;
    out.fuel_pos = *f_target;

    out.set_lox = true;
    out.lox_pos = *l_target;

    return out;
}
