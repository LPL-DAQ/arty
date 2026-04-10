#ifndef APP_THROTTLE_HORNET_H
#define APP_THROTTLE_HORNET_H

#include <cmath>
#include "../ThrottleController.h"

namespace ThrottleHornet {

std::expected<void, Error> tick(ThrottleStateOutput& output, DataPacket& data, const AnalogSensorReadings& analog_sensors);

}

#endif // APP_THROTTLE_HORNET_H
