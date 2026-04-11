#include "TVCRanger.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TVCRanger, LOG_LEVEL_INF);

std::expected<void, Error> TVCRanger::tick(TVCStateOutput& output, DataPacket& data, const AnalogSensorReadings& analog_sensors){
    data.which_tvc_actuator_data = DataPacket_tvc_ranger_data_tag;
    auto& tvc_data = data.tvc_actuator_data.tvc_ranger_data;
    tvc_data.target_x_deg = 0.0f;
    tvc_data.target_y_deg = 0.0f;
    tvc_data.encoder_x_deg = 0.0f;
    tvc_data.encoder_y_deg = 0.0f;
    return {};
}
