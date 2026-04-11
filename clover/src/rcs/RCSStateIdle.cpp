#include "RCSStateIdle.h"

void RCSStateIdle::init() {
    // Initial setup if needed (valves are stopped by the output struct)
}

std::pair<RCSStateOutput, RCSIdleData> RCSStateIdle::tick() {
    RCSStateOutput out{};
    RCSIdleData data{};
    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_IDLE;
    return std::make_pair(out, data);

}
