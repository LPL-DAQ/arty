#include "AbortState.h"

void AbortState::init() {
    // Controller handles actuation now
}

ControllerOutput AbortState::tick(uint32_t current_time, uint32_t entry_time, float default_fuel, float default_lox) {
    ControllerOutput out;

    // Drive valves to nominal safe positions
    out.set_fuel = true;
    out.fuel_pos = default_fuel;

    out.set_lox = true;
    out.lox_pos = default_lox;

    // Run for ~0.5s before allowing state transition back to idle
    if (current_time - entry_time > 500) {
        out.next_state = SystemState_STATE_IDLE;
    } else {
        out.next_state = SystemState_STATE_ABORT;
    }

    return out;
}
