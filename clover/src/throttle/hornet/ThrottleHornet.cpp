#include "ThrottleHornet.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ThrottleHornet, LOG_LEVEL_INF);


std::expected<void, Error> ThrottleHornet::tick(ThrottleStateOutput& output, DataPacket& data, const AnalogSensorReadings& analog_sensors){

    // turn output.thrust to pwm signal sent to motor
    return {};
}



