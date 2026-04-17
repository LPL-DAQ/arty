#ifndef APP_TVC_RANGER_H
#define APP_TVC_RANGER_H

#include <cmath>
#include "clover.pb.h"
#include <expected>
#include "../Error.h"


namespace TVCRangerActuator {

std::expected<void, Error> tick(TVCRangerStateOutput& output, TVCRangerData& actuator_data);

}

#endif // APP_TVC_RANGER_H
