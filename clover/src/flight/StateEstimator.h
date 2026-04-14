#ifndef APP_FLIGHT_STATE_ESTIMATOR_H
#define APP_FLIGHT_STATE_ESTIMATOR_H

#include "clover.pb.h"
#include <cstdint>

namespace StateEstimator {

    struct EstimatedState {
        float pitch_deg = 0.0f;
        float yaw_deg = 0.0f;
        float roll_deg = 0.0f;

        float x_m = 0.0f;
        float y_m = 0.0f;
        float z_m = 0.0f;

        float vx_m_s = 0.0f;
        float vy_m_s = 0.0f;
        float vz_m_s = 0.0f;
    };

    void init();
    void reset();
    void step_control_loop(DataPacket& data);

    const EstimatedState& estimate();

    static inline EstimatedState estimate_ = {};
    static inline uint64_t last_time_ns_ = 0;
    static inline bool has_last_time_ = false;
}

#endif // APP_FLIGHT_STATE_ESTIMATOR_H
