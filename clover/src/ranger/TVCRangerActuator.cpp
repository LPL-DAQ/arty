#include "TVCRangerActuator.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TVCRangerActuator, LOG_LEVEL_INF);

std::expected<void, Error> TVCRangerActuator::tick(float x_pos, float y_pos, TVCRangerData& data){
    // fill in actuator data
    // this is where adit (i think) code goes

    return {};
}
