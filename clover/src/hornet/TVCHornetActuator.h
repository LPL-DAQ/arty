#ifndef APP_TVC_HORNET_H
#define APP_TVC_HORNET_H

#include <cmath>
#include "../TVCController.h"

namespace TVCHornetActuator {

std::expected<void, Error> tick(TVCStateOutput& output, DataPacket& data);

}

#endif // APP_TVC_HORNET_H
