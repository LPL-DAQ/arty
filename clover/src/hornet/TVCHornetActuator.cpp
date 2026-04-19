#include "TVCHornetActuator.h"
#include "../PwmActuator.h"
#include "../ControllerConfig.h"
#include "../LookupTable.h"
#include <algorithm>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TVCHornetActuator, LOG_LEVEL_INF);

std::expected<void, Error> TVCHornetActuator::tick(TVCHornetStateOutput& output, DataPacket& data){
    // Clamp angles to servo range: -90 to +90 degrees
    const float pitch_angle = std::clamp(output.target_pitch, -90.0f, 90.0f);
    const float yaw_angle = std::clamp(output.target_yaw, -90.0f, 90.0f);

    // Convert angle to pulse width
    // Mapping: -90° -> 1000µs, 0° -> 1500µs, +90° -> 2000µs
    const uint32_t pitch_pulse_us = static_cast<uint32_t>(1500.0f + (pitch_angle * 5.55556f));
    const uint32_t yaw_pulse_us = static_cast<uint32_t>(1500.0f + (yaw_angle * 5.555556f));

    auto err = ServoX::tick(pitch_pulse_us);
    if (!err) return err;
    err = ServoY::tick(yaw_pulse_us);
    if (!err) return err;

    data.which_tvc_actuator_data = DataPacket_tvc_hornet_data_tag;
    data.tvc_actuator_data.tvc_hornet_data.pitch_angle = pitch_angle;
    data.tvc_actuator_data.tvc_hornet_data.yaw_angle = yaw_angle;
    return {};
}
