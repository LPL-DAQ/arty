#include "RCSRanger.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RCSRanger, LOG_LEVEL_INF);

std::expected<void, Error> RCSRanger::tick(RCSStateOutput& output, DataPacket& data){
    data.which_rcs_actuator_data = DataPacket_rcs_ranger_data_tag;
    auto& rcs_data = data.rcs_actuator_data.rcs_ranger_data;
    rcs_data.cw_open = false;
    rcs_data.ccw_open = false;
    rcs_data.target_position_deg = 0.0f;
    rcs_data.target_vel_deg_sec = 0.0f;
    return {};
}
