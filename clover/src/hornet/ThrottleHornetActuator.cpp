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

    auto err = MotorCR::set_target_throttle(throttle);
    if (!err) return err;
    data.throttle_percent = throttle;

    err = MotorCR::tick();
    if (!err) return err;

    return {};
}
