#pragma once

#include <cstdint>

namespace Watchdog {

// Initializes and enables the hardware watchdog peripheral.
bool init(uint32_t timeout_ms = 3000);

// Feeds (refreshes) the hardware watchdog timer.
void feed();

// Struct for reporting reboot checks
struct RebootCheckResult {
    bool cause_available = false;
    bool watchdog_reset = false;
    uint32_t cause = 0;
    int read_error = 0;
    int clear_error = 0;
};

// Checks if the cause of a teensy reboot was a watchdog restart.
RebootCheckResult evaluate_reboot_cause();

} // namespace Watchdog