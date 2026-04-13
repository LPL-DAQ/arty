#ifndef APP_RCS_HORNET_H
#define APP_RCS_HORNET_H

#include <cmath>
#include "../RCSController.h"

namespace RCSHornetActuator {

std::expected<void, Error> tick(RCSStateOutput& output, RCSHornetData& data);

}

#endif // APP_RCS_HORNET_H
