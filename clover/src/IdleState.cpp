#include "IdleState.h"

void IdleState::init() {
    // Initial setup if needed (valves are stopped by the output struct)
}

ControllerOutput IdleState::tick() {
    ControllerOutput out{};
    out.set_fuel = false; // Tells the Controller to call stop()
    out.set_lox = false;
    out.next_state = SystemState_STATE_IDLE;
    return out;
}
