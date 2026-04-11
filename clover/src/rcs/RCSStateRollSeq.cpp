#include "RCSStateRollSeq.h"
#include "../ControllerConfig.h"
#include "../LookupTable.h"

#include <algorithm>
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(RCSController, LOG_LEVEL_INF);

namespace {

    float tot_ms;
}  // namespace

void StateRollSeq::init(float total_time_ms)
{
    LOG_INF("Entering Closed Loop RCS Mode");

    tot_ms = total_time_ms;
}

std::pair<RCSStateOutput, RCSRollSequenceData> StateRollSeq::tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time)
{
    RCSStateOutput out{};
    RCSRollSequenceData data{};

    // Populate telemetry data


    // 10. Populate RCSStateOutput

    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_ROLL_SEQ;
    return {out, data};
}

Trace& StateRollSeq::get_trace()
{
    return trace;
}
