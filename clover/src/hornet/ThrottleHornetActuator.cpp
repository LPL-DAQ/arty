#include "ThrottleHornetActuator.h"
#include "../PwmActuator.h"
#include "../ControllerConfig.h"
#include "../LookupTable.h"
#include <algorithm>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ThrottleHornetActuator, LOG_LEVEL_INF);


std::expected<void, Error> ThrottleHornetActuator::tick(ThrottleHornetStateOutput& output, ThrottleHornetData& data){
    float throttle = 0.0f;
    if (output.power_on) {
        throttle = std::clamp(output.throttle_percent, 0.0f, 1.0f);
    }

    // Convert throttle to pulse width: 0.0 -> 1000µs, 1.0 -> 2000µs
    const uint32_t pulse_us = static_cast<uint32_t>(1000.0f + (throttle * 1000.0f));
    
    auto err = MotorTop::tick(pulse_us);
    if (!err) return err;
    err = MotorBottom::tick(pulse_us);
    if (!err) return err;

    data.throttle_percent = throttle;
    return {};
}
