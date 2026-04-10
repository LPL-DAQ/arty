#include "ThrottleStateIdle.h"

void ThrottleStateIdle::init() {
    // Initial setup if needed (valves are stopped by the output struct)
}

std::pair<ThrottleStateOutput, ThrottleIdleData> ThrottleStateIdle::tick() {
    ThrottleStateOutput out{};
    ThrottleIdleData data{};
    out.has_power_on = true;
    out.power_on = true;
    out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    return std::make_pair(out, data);

}
