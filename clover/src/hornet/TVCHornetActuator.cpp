#include "TVCHornetActuator.h"
#include "../PwmActuator.h"
#include "../ControllerConfig.h"
#include "../LookupTable.h"
#include <algorithm>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TVCHornetActuator, LOG_LEVEL_INF);

std::expected<void, Error> TVCHornetActuator::tick(TVCHornetStateOutput& output, DataPacket& data){
    const float pitch_angle = std::clamp(output.target_pitch, ServoL::get_angle_min(), ServoL::get_angle_max());
    const float yaw_angle = std::clamp(output.target_yaw, ServoR::get_angle_min(), ServoR::get_angle_max());

    auto err = ServoL::set_target_angle(pitch_angle);
    if (!err) return err;
    err = ServoR::set_target_angle(yaw_angle);
    if (!err) return err;

    err = ServoL::tick();
    if (!err) return err;
    err = ServoR::tick();
    if (!err) return err;

    data.which_tvc_actuator_data = DataPacket_tvc_hornet_data_tag;
    data.tvc_actuator_data.tvc_hornet_data.pitch_angle = pitch_angle;
    data.tvc_actuator_data.tvc_hornet_data.yaw_angle = yaw_angle;
    return {};
}
