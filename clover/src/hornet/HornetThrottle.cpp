#include "HornetThrottle.h"
#include "MutexGuard.h"
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(hornet_throttle_lock);

/// Reset internal state before an active control trace
void HornetThrottle::reset()
{
    MutexGuard hornet_throttle_guard{&hornet_throttle_lock};
    // TODO
}

/// Generate a comomand for the main propeller.
std::expected<std::tuple<float, HornetThrottleMetrics>, Error> HornetThrottle::tick(float thrust_command_lbf)
{
    MutexGuard hornet_throttle_guard{&hornet_throttle_lock};

    float main_propeller_command = -67.67f;
    HornetThrottleMetrics metrics = HornetThrottleMetrics_init_default;
    return {{main_propeller_command, metrics}};
}
