#ifndef APP_THROTTLE_HORNET_H
#define APP_THROTTLE_HORNET_H

#include <cmath>
#include "../Error.h"
#include "../ThrottleController.h"

namespace ThrottleHornetActuator {
    
std::expected<void, Error> tick(ThrottleHornetStateOutput& output, DataPacket& data);

}
#endif // APP_THROTTLE_HORNET_H
