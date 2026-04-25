#ifndef APP_FLIGHT_STATE_ESTIMATOR_H
#define APP_FLIGHT_STATE_ESTIMATOR_H

#include "Error.h"
#include "clover.pb.h"
#include <cstdint>
#include <optional>

namespace StateEstimator {

    void init();
    void reset();
    std::optional<EstimatedState> estimate(
        LidarReading& lidar_1,
        LidarReading& lidar_2,
        ImuReading& imu,
        GnssReadings& gnss
    );


}

#endif // APP_FLIGHT_STATE_ESTIMATOR_H
