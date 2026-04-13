#include "ThrottleHornetActuator.h"
#include "ThrottleValve.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ThrottleHornetActuator, LOG_LEVEL_INF);


std::expected<void, Error> ThrottleHornetActuator::tick(ThrottleHornetStateOutput& output, ThrottleHornetActuatorData data){

    return {};
}
