#include "TVCStateIdle.h"

void TVCStateIdle::init() {
    // Initial setup if needed (valves are stopped by the output struct)
}

std::pair<TVCControllerOutput, TVCIdleData> TVCStateIdle::tick() {
    TVCControllerOutput out{};
    TVCIdleData data{};
    out.next_state = TVCState_TVC_STATE_IDLE;
    return std::make_pair(out, data);

}
