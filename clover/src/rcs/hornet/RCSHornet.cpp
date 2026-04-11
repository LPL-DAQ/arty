#include "RCSHornet.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RCSHornet, LOG_LEVEL_INF);

std::expected<void, Error> RCSHornet::tick(RCSStateOutput& output, DataPacket& data, const AnalogSensorReadings& analog_sensors){
    data.which_rcs_actuator_data = DataPacket_rcs_hornet_data_tag;
    auto& rcs_data = data.rcs_actuator_data.rcs_hornet_data;
    rcs_data.cw_power_on = false;
    rcs_data.cw_power_off = false;
    rcs_data.target_position_deg = 0.0f;
    rcs_data.target_vel_deg_sec = 0.0f;
    return {};
}
