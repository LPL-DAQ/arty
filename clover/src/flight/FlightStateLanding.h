#ifndef APP_FLIGHT_STATE_LANDING_H
#define APP_FLIGHT_STATE_LANDING_H

#include "clover.pb.h"
#include <utility>

namespace FlightStateLanding {
    void init();
    std::pair<FlightStateOutput, FlightLandingData> tick(int64_t current_time, int64_t start_time);
}

#endif  // APP_FLIGHT_STATE_LANDING_H
