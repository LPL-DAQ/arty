#pragma once

#include "Error.h"
#include <expected>

#ifdef CONFIG_BLINK_LED

namespace BlinkLed {
std::expected<void, Error> init();
}

#endif  // CONFIG_BLINK_LED
