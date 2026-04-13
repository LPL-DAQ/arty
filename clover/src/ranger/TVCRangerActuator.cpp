#include "TVCRangerActuator.h"
#include "../ControllerConfig.h"
#include "../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TVCRangerActuator, LOG_LEVEL_INF);

std::expected<void, Error> tick(TVCRangerStateOutput& output, DataPacket& data){
    // fill in actuator data
    // this is where adit (i think) code goes

    return {};
}
