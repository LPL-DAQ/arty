#include "StateIdle.h"

void StateIdle::init() {
    // Initial setup if needed (valves are stopped by the output struct)
}

std::pair<ControllerOutput, IdleData> StateIdle::tick() {
    ControllerOutput out{};
    IdleData data{};
    out.set_fuel = false; // Tells the Controller to call stop()
    out.set_lox = false;
    out.next_state = SystemState_STATE_IDLE;
    return std::make_pair(out, data);

}
