#ifndef APP_THROTTLE_HORNET_H
#define APP_THROTTLE_HORNET_H

#include <cmath>
#include "clover.pb.h"
#include "../Error.h"
#include <expected>

namespace ThrottleHornetActuator {

std::expected<void, Error> tick(ThrottleHornetStateOutput& output, ThrottleHornetData& data);

}
#endif // APP_THROTTLE_HORNET_H
