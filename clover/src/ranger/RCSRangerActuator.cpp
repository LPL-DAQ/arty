#include "RCSRangerActuator.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RCSRangerActuator, LOG_LEVEL_INF);

std::expected<void, Error> RCSRangerActuator::tick(bool cw_open, bool ccw_open, RCSRangerData& data){
    // fill in actuator data
    // this is where adit (i think) code goes

    return {};
}
