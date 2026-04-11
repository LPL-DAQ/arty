#include "RCSStateValveSeq.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(RCSController, LOG_LEVEL_INF);

static Trace trace;
static bool has_trace;
static float total_time;

void RCSStateValveSeq::init(bool has_trace_arg, float total_time_ms)
{
    // Timer is reset in handle_start_sequence before entering

    has_trace = has_trace_arg;
    total_time = total_time_ms;
}

std::pair<RCSStateOutput, RCSValveSequenceData> RCSStateValveSeq::tick(int64_t current_time, int64_t start_time)
{
    RCSStateOutput out{};
    RCSValveSequenceData data{};
    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_VALVE_SEQ;  // Assume we stay in this state by default

    float dt = current_time - start_time;

    if (has_trace) {
        auto target = trace.sample(dt);
        if (!target) {
            LOG_ERR("Failed to sample valve trace: %s", target.error().build_message().c_str());
            out.next_state = RCSState_RCS_STATE_IDLE;
            return {out, data};
        }

        float value = *target;
        if (value > 0.0f) {
            out.CW = true;
            out.CCW = false;
        } else if (value < 0.0f) {
            out.CW = false;
            out.CCW = true;
        }
    }

    if (has_trace && dt >= total_time) {
        LOG_INF("Done RCS valve trace sequence, dt was %f", static_cast<double>(dt));
        out.next_state = RCSState_RCS_STATE_IDLE;
    }

    return std::make_pair(out, data);
}

Trace& RCSStateValveSeq::get_trace()
{
    return trace;
}


