#ifndef APP_RCS_STATE_VALVE_SEQ_H
#define APP_RCS_STATE_VALVE_SEQ_H

#include "RCSController.h"
#include "Trace.h"

namespace RCSStateValveSeq {
    void init(bool has_trace, float total_time_ms);
    std::pair<RCSControllerOutput, RCSValveSequenceData> tick(int64_t current_time, int64_t start_time);
    Trace& get_trace();

}

#endif // APP_RCS_STATE_VALVE_SEQ_H
