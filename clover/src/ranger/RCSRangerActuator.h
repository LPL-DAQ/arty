#ifndef APP_RCS_RANGER_H
#define APP_RCS_RANGER_H

#include <cmath>
#include "clover.pb.h"
#include <expected>
#include "../Error.h"

namespace RCSRangerActuator {

std::expected<void, Error> tick(RCSRangerStateOutput& output, RCSRangerData& data);

}

#endif // APP_RCS_RANGER_H
