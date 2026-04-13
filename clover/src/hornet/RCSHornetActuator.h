#ifndef APP_RCS_HORNET_H
#define APP_RCS_HORNET_H

#include <cmath>
#include "clover.pb.h"
#include <expected>
#include "../Error.h"

namespace RCSHornetActuator {

std::expected<void, Error> tick(RCSHornetStateOutput& output, RCSHornetData& data);

}

#endif // APP_RCS_HORNET_H
