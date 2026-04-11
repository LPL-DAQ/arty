#include "RCSStateRollSeq.h"
#include "../ControllerConfig.h"
#include "../LookupTable.h"

#include <algorithm>
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(RCSController, LOG_LEVEL_INF);

namespace {
    bool has_trace = false;
    float total_time = 0.0f;
}  // namespace

void StateRollSeq::init(float total_time_ms)
{
    LOG_INF("Entering Closed Loop RCS Mode");

    has_trace = true;
    total_time = total_time_ms;
}

std::pair<RCSStateOutput, RCSRollSequenceData> StateRollSeq::tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time)
{
    RCSStateOutput out{};
    RCSRollSequenceData data{};

    float dt = current_time - start_time;

    if (has_trace) {
        auto target = trace.sample(dt);
        if (!target) {
            LOG_ERR("Failed to sample roll trace: %s", target.error().build_message().c_str());
            out.next_state = RCSState_RCS_STATE_IDLE;
            out.CW = false;
            out.CCW = false;
            return {out, data};
        }

        // placeholder control code
        float value = *target;
        if (value > 0.0f) {
            out.CW = true;
            out.CCW = false;
        } else if (value < 0.0f) {
            out.CW = false;
            out.CCW = true;
        } else {
            out.CW = false;
            out.CCW = false;
        }
    } else {
        out.CW = false;
        out.CCW = false;
    }

    out.next_state = RCSState_RCS_STATE_ROLL_SEQ;
    if (has_trace && dt >= total_time) {
        LOG_INF("Done RCS roll trace sequence, dt was %f", static_cast<double>(dt));
        out.next_state = RCSState_RCS_STATE_IDLE;
    }

    return {out, data};
}

Trace& StateRollSeq::get_trace()
{
    return trace;
}
