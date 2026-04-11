#ifndef APP_RCS_HORNET_H
#define APP_RCS_HORNET_H

#include <cmath>
#include "../RCSController.h"

namespace RCSHornet {

std::expected<void, Error> tick(RCSStateOutput& output, DataPacket& data, const AnalogSensorReadings& analog_sensors);

}

#endif // APP_RCS_HORNET_H
