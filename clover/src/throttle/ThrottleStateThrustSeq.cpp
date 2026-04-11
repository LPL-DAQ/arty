#include "ThrottleStateThrustSeq.h"
#include "../ControllerConfig.h"
#include "../LookupTable.h"

#include <algorithm>
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(ThrottleController, LOG_LEVEL_INF);


void StateThrustSeq::init(float total_time_ms)
{
    LOG_INF("Entering Closed Loop Throttle Mode");

    tot_ms = total_time_ms;
}

// needs float target_thrust_lbf from Sample
std::pair<ThrottleStateOutput, ThrottleThrustSequenceData> StateThrustSeq::tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time)
{
    ThrottleStateOutput out{};
    ThrottleThrustSequenceData data{};

    // get desired thrust sample
    float elapsed_time = current_time - start_time;
    if (elapsed_time > tot_ms) {
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
        return std::make_pair(out, data);
    }
    auto target_result = trace.sample(elapsed_time);
    if (!target_result) {
        auto msg = target_result.error().build_message();
        LOG_ERR("Failed to sample thrust_trace: %s", msg.c_str());
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
        return std::make_pair(out, data);
    }
    float target_thrust_lbf = *target_result;
    out.has_thrust = true;
    out.thrust = target_thrust_lbf;
    out.power_on = true;
    out.next_state = ThrottleState_THROTTLE_STATE_THRUST_SEQ;

    data.target_thrust = target_thrust_lbf;
    data.thrust_error = 0.0f;

    return {out, data};
}

Trace& StateThrustSeq::get_trace()
{
    return trace;
}
