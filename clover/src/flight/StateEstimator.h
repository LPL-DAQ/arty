#ifndef APP_FLIGHT_STATE_ESTIMATOR_H
#define APP_FLIGHT_STATE_ESTIMATOR_H

#include "clover.pb.h"
#include <cstdint>

namespace StateEstimator {

    void init();
    void reset();
    void step_control_loop(DataPacket& data);

    const EstimatedState& estimate();

    static inline EstimatedState estimate_ = EstimatedState_init_default;
    static inline uint64_t last_time_ns_ = 0;
    static inline bool has_last_time_ = false;
}

#endif // APP_FLIGHT_STATE_ESTIMATOR_H
