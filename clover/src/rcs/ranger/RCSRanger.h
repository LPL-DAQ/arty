#ifndef APP_RCS_RANGER_H
#define APP_RCS_RANGER_H

#include <cmath>
#include "../RCSController.h"

namespace RCSRanger {

std::expected<void, Error> tick(RCSStateOutput& output, DataPacket& data);

}

#endif // APP_RCS_RANGER_H
