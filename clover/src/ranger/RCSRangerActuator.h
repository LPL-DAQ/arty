#ifndef APP_RCS_RANGER_H
#define APP_RCS_RANGER_H

#include <cmath>
#include "../RCSController.h"

namespace RCSRangerActuator {

std::expected<void, Error> tick(RCSStateOutput& output, RCSRangerData& data);

}

#endif // APP_RCS_RANGER_H
