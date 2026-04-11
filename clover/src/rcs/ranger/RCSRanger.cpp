#include "RCSRanger.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RCSRanger, LOG_LEVEL_INF);

std::expected<void, Error> RCSRanger::tick(RCSStateOutput& output, DataPacket& data, const AnalogSensorReadings& analog_sensors){
    // Convert output.next_state and other state output fields into actuator commands.
    return {};
}
