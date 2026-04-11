#ifndef APP_TVC_STATE_SEQ_H
#define APP_TVC_STATE_SEQ_H

#include "TVCController.h"
#include "Trace.h"

namespace TVCStateSeq {
    void init(bool has_trace, float total_time_ms);
    std::pair<TVCStateOutput, TVCSequenceData> tick(int64_t current_time, int64_t start_time);
    Trace& get_x_trace();
    Trace& get_y_trace();
}

#endif // APP_TVC_STATE_SEQ_H
