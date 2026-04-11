#include "TVCStateIdle.h"

void TVCStateIdle::init() {
    // Initial setup if needed (valves are stopped by the output struct)
}

std::pair<TVCStateOutput, TVCIdleData> TVCStateIdle::tick() {
    TVCStateOutput out{};
    TVCIdleData data{};
    out.next_state = TVCState_TVC_STATE_IDLE;
    return std::make_pair(out, data);

}
