#include "RCSHornetActuator.h"
#include "../PwmActuator.h"
#include "../ControllerConfig.h"
#include "../LookupTable.h"
#include <algorithm>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RCSHornetActuator, LOG_LEVEL_INF);

std::expected<void, Error> RCSHornetActuator::tick(RCSHornetStateOutput& output, RCSHornetData& data){
    const float cw_throttle  = output.CW  ? 1.0f : 0.0f;
    const float ccw_throttle = output.CCW ? 1.0f : 0.0f;

    auto err = MotorBeta1::set_target_throttle(cw_throttle);
    if (!err) return err;
    err = MotorBeta2::set_target_throttle(ccw_throttle);
    if (!err) return err;
    err = MotorBeta3::set_target_throttle(cw_throttle);
    if (!err) return err;
    err = MotorBeta4::set_target_throttle(ccw_throttle);
    if (!err) return err;

    err = MotorBeta1::tick();
    if (!err) return err;
    err = MotorBeta2::tick();
    if (!err) return err;
    err = MotorBeta3::tick();
    if (!err) return err;
    err = MotorBeta4::tick();
    if (!err) return err;

    data.cw_power_on = output.CW;
    data.cw_power_off = !output.CW;
    return {};
}

