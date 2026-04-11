#ifndef APP_FLIGHT_STATE_TAKEOFF_H
#define APP_FLIGHT_STATE_TAKEOFF_H

#include "clover.pb.h"
#include <utility>

namespace FlightStateTakeoff {
    void init();
    std::pair<FlightStateOutput, FlightTakeoffData> tick(int64_t current_time, int64_t start_time);
}

#endif  // APP_FLIGHT_STATE_TAKEOFF_H
