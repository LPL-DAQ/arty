#include "RCSHornetActuator.h"
#include "../ControllerConfig.h"
#include "../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RCSHornetActuator, LOG_LEVEL_INF);

std::expected<void, Error> RCSHornetActuator::tick(RCSHornetStateOutput& output, RCSHornetData& data){
    // fill in actuator data
    // this is where adit (i think) code goes

    
    return {};
}

