#include "StateEstimator.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(StateEstimator, LOG_LEVEL_INF);

namespace StateEstimator {

void init()
{
    reset();
}

void reset()
{
    estimate_ = {};
    last_time_ns_ = 0;
    has_last_time_ = false;
}

void step_control_loop(DataPacket& data)
{
    estimate_ = EstimatedState_init_default;
    data.has_estimated_state = true;
    data.estimated_state = estimate_;
}

const EstimatedState& estimate()
{
    return estimate_;
}

} // namespace StateEstimator
