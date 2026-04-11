#include "TVCHornet.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TVCHornet, LOG_LEVEL_INF);

std::expected<void, Error> TVCHornet::tick(TVCStateOutput& output, DataPacket& data){

    // from here, we interaface with the motor (prabhu code) + thrust characterization
    auto& tvc_data = data.tvc_actuator_data.tvc_hornet_data;
    tvc_data.x_pos = 0.0f;
    tvc_data.y_pos = 0.0f;
    return {};
}
