#include "RCSHornetActuator.h"
#include "../PwmActuator.h"
#include "../ControllerConfig.h"
#include "../LookupTable.h"
#include "../PwmActuator.h"
#include <algorithm>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RCSHornetActuator, LOG_LEVEL_INF);

std::expected<void, Error> RCSHornetActuator::tick(RCSHornetStateOutput& output, RCSHornetData& data){
    const float throttle_value = 1.0f;
    const float cw_throttle  = output.CW  ? throttle_value : 0.0f;
    const float ccw_throttle = output.CCW ? throttle_value : 0.0f;

    // Convert throttle to pulse width: 0.0 -> 1000µs, 1.0 -> 2000µs
    const uint32_t cw_pulse_us = static_cast<uint32_t>(1000.0f + (cw_throttle * 1000.0f));
    const uint32_t ccw_pulse_us = static_cast<uint32_t>(1000.0f + (ccw_throttle * 1000.0f));

    // TODO: Check which actually corresponds to what
    auto err = BetaTop::tick(cw_pulse_us);
    if (!err) return err;
    err = BetaBottom::tick(ccw_pulse_us);
    if (!err) return err;
    err = BetaCW::tick(cw_pulse_us);
    if (!err) return err;
    err = BetaCCW::tick(ccw_pulse_us);
    if (!err) return err;

    data.cw_power_on = output.CW;
    data.cww_power_on = output.CCW;
    return {};
}

