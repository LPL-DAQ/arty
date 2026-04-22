#ifndef APP_FLIGHT_STATE_ESTIMATOR_H
#define APP_FLIGHT_STATE_ESTIMATOR_H

#include "Error.h"
#include "clover.pb.h"
#include <cstdint>
#include <optional>

namespace StateEstimator {

    void init();
    void reset();
    std::optional<EstimatedState> estimate(DataPacket& data);


}

#endif // APP_FLIGHT_STATE_ESTIMATOR_H
