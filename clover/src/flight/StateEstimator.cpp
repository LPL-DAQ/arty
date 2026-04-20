#include "StateEstimator.h"
#include "Error.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(StateEstimator, LOG_LEVEL_INF);

namespace  {

    static inline EstimatedState current_estimate = EstimatedState_init_default;
    static inline uint64_t last_time_ms = 0;
    static inline bool has_last_time = false;
}

void StateEstimator::init()
{
    reset();
}

void StateEstimator::reset()
{
    current_estimate = EstimatedState_init_default;
    current_estimate.R_WB.qw = 1.0f; // to get identity q
    last_time_ms = 0;
    has_last_time = false;
}

std::optional<EstimatedState> StateEstimator::estimate(DataPacket& data)
{
    // TODO: Fill in when the real sensors exist

    // GNCSensorReadings& sensors = data.gnc_sensors;
    // estimate.position = sensors.javad_position;
    //     // edit z for lidar
    // estimate.velocity = sensors.javad_velocity;
    // estimate.R_WB = sensors.vn_q;
    // data.estimated_state = estimate;
    // // TODO: some checking about when the last update from sensors were and whatnot
    // last_time_ms = k_uptime_get();
    // has_last_time = true;
    return current_estimate;

}

