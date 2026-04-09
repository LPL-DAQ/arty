#include "RCSStateIdle.h"

void RCSStateIdle::init() {
    // Initial setup if needed (valves are stopped by the output struct)
}

std::pair<RCSControllerOutput, RCSIdleData> RCSStateIdle::tick() {
    RCSControllerOutput out{};
    RCSIdleData data{};
    out.next_state = RCSState_RCS_STATE_IDLE;
    return std::make_pair(out, data);

}
