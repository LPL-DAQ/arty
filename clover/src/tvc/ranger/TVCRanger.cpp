#include "TVCRanger.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TVCRanger, LOG_LEVEL_INF);

std::expected<void, Error> TVCRanger::tick(TVCStateOutput& output, DataPacket& data){

    // this is where adit (i think) code goes

    data.which_tvc_actuator_data = DataPacket_tvc_ranger_data_tag;
    auto& tvc_data = data.tvc_actuator_data.tvc_ranger_data;
    tvc_data.x_pos = 0.0f;
    tvc_data.y_pos = 0.0f;
    return {};
}
