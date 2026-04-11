#include "RCSHornet.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RCSHornet, LOG_LEVEL_INF);

std::expected<void, Error> RCSHornet::tick(RCSStateOutput& output, DataPacket& data){
    data.which_rcs_actuator_data = DataPacket_rcs_hornet_data_tag;
    auto& rcs_data = data.rcs_actuator_data.rcs_hornet_data;
    rcs_data.cw_power_on = false;
    rcs_data.cw_power_off = false;

    return {};
}
