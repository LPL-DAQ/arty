#include "ThrottleHornet.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ThrottleHornet, LOG_LEVEL_INF);

std::expected<void, Error> ThrottleHornet::tick(ThrottleStateOutput& output, DataPacket& data){

    // turn output.thrust to pwm signal sent to motor
    return {};
}

void ThrottleHornet::set_throttle_actuator_data_tag(DataPacket& data)
{
    data.which_throttle_actuator_data = DataPacket_throttle_hornet_data_tag;
}

void ThrottleHornet::init_state(ThrottleState new_state)
{

}


