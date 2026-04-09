#include "TVCStateAbort.h"

void TVCStateAbort::init() {
    // Controller handles actuation now
}

std::pair<TVCControllerOutput, RCSAbortData> TVCStateAbort::tick(uint32_t current_time, uint32_t entry_time) {
    TVCControllerOutput out{};
    RCSAbortData abort_data{};


    // Run for ~0.5s before allowing state transition back to idle
    if (current_time - entry_time > 500) {
        out.next_state = TVCState_TVC_STATE_IDLE;
    } else {
        out.next_state = TVCState_TVC_STATE_ABORT;
    }

    return std::make_pair(out, abort_data);
}
