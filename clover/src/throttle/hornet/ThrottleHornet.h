#ifndef APP_THROTTLE_HORNET_H
#define APP_THROTTLE_HORNET_H

#include <cmath>
#include "../Error.h"
#include "../ThrottleController.h"

namespace ThrottleHornet {

std::expected<void, Error> tick(ThrottleStateOutput& output, DataPacket& data);

void set_throttle_actuator_data_tag(DataPacket& data);
void init_thrust_sequence(float total_time_ms);
}

#endif // APP_THROTTLE_HORNET_H
