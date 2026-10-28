#pragma once

#include <cstdint>

namespace Watchdog {

// Initializes and enables the hardware watchdog peripheral.
bool init(uint32_t timeout_ms = 3000);

// Feeds (refreshes) the hardware watchdog timer.
void feed();

} // namespace Watchdog