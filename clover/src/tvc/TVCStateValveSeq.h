#ifndef APP_TVC_STATE_VALVE_SEQ_H
#define APP_TVC_STATE_VALVE_SEQ_H

#include "TVCController.h"
#include "Trace.h"

namespace TVCStateValveSeq {
    void init(bool has_trace, float total_time_ms);
    std::pair<TVCControllerOutput, RCSValveSequenceData> tick(int64_t current_time, int64_t start_time);
    Trace& get_trace();

}

#endif // APP_TVC_STATE_VALVE_SEQ_H
