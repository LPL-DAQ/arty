#include "TVCStateValveSeq.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(TVCController, LOG_LEVEL_INF);

static Trace trace;
static bool has_trace;
static float total_time;

void TVCStateValveSeq::init(bool has_trace, float total_time_ms)
{
    // Timer is reset in handle_start_sequence before entering

    has_trace = has_trace;
    total_time = total_time_ms;
}

std::pair<TVCControllerOutput, RCSValveSequenceData> TVCStateValveSeq::tick(int64_t current_time, int64_t start_time)
{
    TVCControllerOutput out;
    RCSValveSequenceData data{};
    out.next_state = RCSState_RCS_STATE_VALVE_SEQ;  // Assume we stay in this state by default

    return std::make_pair(out, data);
}

Trace& RCSStateValveSeq::get_trace()
{
    return trace;
}


