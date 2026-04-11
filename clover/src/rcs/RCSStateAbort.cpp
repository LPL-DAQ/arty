#include "RCSStateAbort.h"

void RCSStateAbort::init() {
    // Controller handles actuation now
}

std::pair<RCSStateOutput, RCSAbortData> RCSStateAbort::tick(uint32_t current_time, uint32_t entry_time) {
    RCSStateOutput out{};
    RCSAbortData abort_data{};


    // Run for ~0.5s before allowing state transition back to idle
    if (current_time - entry_time > 500) {
        out.next_state = RCSState_RCS_STATE_IDLE;
    } else {
        out.next_state = RCSState_RCS_STATE_ABORT;
    }

    out.CW = false;
    out.CCW = false;
    return std::make_pair(out, abort_data);
}
