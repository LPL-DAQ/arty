#include "StateRollSeq.h"
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

std::pair<RCSControllerOutput, RCSRollSequenceData> StateRollSeq::tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time)
{
    RCSControllerOutput out{};
    RCSRollSequenceData data{};

    // Populate telemetry data


    // 10. Populate RCSControllerOutput


    return {out, data};
}

Trace& StateRollSeq::get_trace()
{
    return trace;
}
