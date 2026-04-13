#ifndef APP_TVC_RANGER_H
#define APP_TVC_RANGER_H

#include <cmath>
#include "../TVCController.h"

namespace TVCRangerActuator {

std::expected<void, Error> tick(TVCStateOutput& output, DataPacket& data);

}

#endif // APP_TVC_RANGER_H
