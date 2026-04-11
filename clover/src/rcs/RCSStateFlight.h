#ifndef APP_STATE_RCS_FLIGHT_H
#define APP_STATE_RCS_FLIGHT_H

#include "clover.pb.h"
#include <utility>

namespace RCSStateFlight {
    void init();
    std::pair<RCSStateOutput, RCSFlightData> tick();
}

#endif  // APP_STATE_RCS_FLIGHT_H
