#ifndef APP_FLIGHT_STATE_IDLE_H
#define APP_FLIGHT_STATE_IDLE_H

#include "clover.pb.h"
#include <utility>

namespace FlightStateIdle {
    void init();
    std::pair<FlightStateOutput, FlightIdleData> tick();
}

#endif  // APP_FLIGHT_STATE_IDLE_H
