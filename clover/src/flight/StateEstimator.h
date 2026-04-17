#ifndef APP_FLIGHT_STATE_ESTIMATOR_H
#define APP_FLIGHT_STATE_ESTIMATOR_H

#include "clover.pb.h"
#include <cstdint>

namespace StateEstimator {

    void init();
    void reset();
    void step_control_loop(DataPacket& data);


}

#endif // APP_FLIGHT_STATE_ESTIMATOR_H
