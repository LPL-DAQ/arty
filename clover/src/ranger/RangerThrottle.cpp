#include "RangerThrottle.h"
#include "MutexGuard.h"
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(ranger_throttle_lock);

/// Reset internal state before an active control trace
void RangerThrottle::reset()
{
    MutexGuard ranger_throttle_guard{&ranger_throttle_lock};
    // TODO
}

/// Generate a comomand for the fuel and lox valve positions in degrees.
std::expected<std::tuple<ThrottleValveCommand, ThrottleValveCommand, RangerThrottleMetrics>, Error> RangerThrottle::tick(float thrust_command_N)
{
    MutexGuard ranger_throttle_guard{&ranger_throttle_lock};

    float fuel_valve_command_deg = -67.67f;
    float lox_valve_command_deg = -67.67f;
    RangerThrottleMetrics metrics = RangerThrottleMetrics_init_default;
    return {
        {ThrottleValveCommand{.enable = true, .set_pos = false, .target_deg = fuel_valve_command_deg},
         ThrottleValveCommand{.enable = true, .set_pos = false, .target_deg = lox_valve_command_deg},
         metrics}};
}
