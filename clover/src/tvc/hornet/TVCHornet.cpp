#include "TVCHornet.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TVCHornet, LOG_LEVEL_INF);

std::expected<void, Error> TVCHornet::tick(TVCStateOutput& output, DataPacket& data, const AnalogSensorReadings& analog_sensors){
    // Convert output.next_state and other state output fields into actuator commands.
    return {};
}
