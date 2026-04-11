#ifndef APP_TVC_RANGER_H
#define APP_TVC_RANGER_H

#include <cmath>
#include "../TVCController.h"

namespace TVCRanger {

std::expected<void, Error> tick(TVCStateOutput& output, DataPacket& data, const AnalogSensorReadings& analog_sensors);

}

#endif // APP_TVC_RANGER_H
