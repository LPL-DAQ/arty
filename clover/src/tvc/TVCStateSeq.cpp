#include "TVCStateSeq.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(TVCController, LOG_LEVEL_INF);

static Trace x_trace;
static Trace y_trace;
static bool has_trace;
static float total_time;

void TVCStateSeq::init(bool has_trace, float total_time_ms)
{
    // Timer is reset in handle_start_sequence before entering

    has_trace = has_trace;
    total_time = total_time_ms;
}

std::pair<TVCControllerOutput, TVCSequenceData> TVCStateSeq::tick(int64_t current_time, int64_t start_time)
{
    TVCControllerOutput out;
    TVCSequenceData data{};
    out.next_state = TVCState_TVC_STATE_TRACE;  // Assume we stay in this state by default

    // TODO: implement the sequence logic
    // For now, set dummy values
    data.x_angle_deg = 0.0f;
    data.y_angle_deg = 0.0f;

    return std::make_pair(out, data);
}

Trace& TVCStateSeq::get_x_trace()
{
    return x_trace;
}

Trace& TVCStateSeq::get_y_trace()
{
    return y_trace;
}


