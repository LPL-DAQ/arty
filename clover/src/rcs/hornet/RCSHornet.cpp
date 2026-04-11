#include "RCSHornet.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RCSHornet, LOG_LEVEL_INF);

std::expected<void, Error> RCSHornet::tick(RCSStateOutput& output, DataPacket& data, const AnalogSensorReadings& analog_sensors){
    // Convert output.next_state and other state output fields into actuator commands.
    return {};
}
