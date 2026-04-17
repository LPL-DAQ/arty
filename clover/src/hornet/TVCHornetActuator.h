#ifndef APP_TVC_HORNET_H
#define APP_TVC_HORNET_H

#include <cmath>
#include "clover.pb.h"
#include <expected>
#include "../Error.h"

namespace TVCHornetActuator {

std::expected<void, Error> tick(TVCHornetStateOutput& output, DataPacket& data);

}

#endif // APP_TVC_HORNET_H
