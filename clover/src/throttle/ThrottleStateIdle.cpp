#include "ThrottleStateIdle.h"

void ThrottleStateIdle::init() {
    // Initial setup if needed (valves are stopped by the output struct)
}

std::pair<ThrottleControllerOutput, ThrottleIdleData> ThrottleStateIdle::tick() {
    ThrottleControllerOutput out{};
    ThrottleIdleData data{};
    out.set_fuel = false; // Tells the Controller to call stop()
    out.set_lox = false;
    out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    return std::make_pair(out, data);

}
