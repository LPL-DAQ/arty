#include "TVCStateSeq.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(TVCController, LOG_LEVEL_INF);

static Trace x_trace;
static Trace y_trace;
static bool has_trace;
static float total_time;

void TVCStateSeq::init(bool has_trace_arg, float total_time_ms)
{
    // Timer is reset in handle_start_sequence before entering

    has_trace = has_trace_arg;
    total_time = total_time_ms;
}

std::pair<TVCStateOutput, TVCSequenceData> TVCStateSeq::tick(int64_t current_time, int64_t start_time)
{
    TVCStateOutput out{};
    TVCSequenceData data{};
    out.next_state = TVCState_TVC_STATE_TRACE;  // Assume we stay in this state by default

    float dt = current_time - start_time;

    if (has_trace) {
        auto x_target = x_trace.sample(dt);
        if (!x_target) {
            LOG_ERR("Failed to sample X trace: %s", x_target.error().build_message().c_str());
            out.next_state = TVCState_TVC_STATE_IDLE;
            return std::make_pair(out, data);
        }

        auto y_target = y_trace.sample(dt);
        if (!y_target) {
            LOG_ERR("Failed to sample Y trace: %s", y_target.error().build_message().c_str());
            out.next_state = TVCState_TVC_STATE_IDLE;
            return std::make_pair(out, data);
        }

        out.target_x = *x_target;
        out.target_y = *y_target;
        data.x_angle_deg = *x_target;
        data.y_angle_deg = *y_target;
    }

    if (has_trace && dt >= total_time) {
        LOG_INF("Done TVC trace sequence, dt was %f", static_cast<double>(dt));
        out.next_state = TVCState_TVC_STATE_IDLE;
    }

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


