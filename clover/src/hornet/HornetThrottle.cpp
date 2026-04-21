#include "HornetThrottle.h"
#include "../MutexGuard.h"
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(hornet_throttle_lock);

/// Reset internal state before an active control trace
void HornetThrottle::reset()
{
    MutexGuard hornet_throttle_guard{&hornet_throttle_lock};
    // TODO
}

/// Generate a comomand for the main propeller.
std::expected<std::tuple<float, HornetThrottleMetrics>, Error> HornetThrottle::tick(float thrust_command_N)
{
    MutexGuard hornet_throttle_guard{&hornet_throttle_lock};
    HornetThrottleMetrics metrics = HornetThrottleMetrics_init_default;
    float best_fit_output;
    float throttle_percent;
    if (thrust_command_N < 44.044f){ // below 10 lbf (only ever during landing and takeoff sequences)
        best_fit_output = (thrust_command_N / 69.911f );
        throttle_percent = std::clamp(best_fit_output, 0.0f, 0.63f);

    } else { // the actual operating range
        best_fit_output = (thrust_command_N + 32.5271f) / 121.53f;
        throttle_percent = std::clamp(best_fit_output, 0.63f, 1.0f);
    }


    // Convert throttle to pulse width: 0.0 -> 1000µs, 1.0 -> 2000µs
    const uint32_t pulse_us = static_cast<uint32_t>(1000.0f + (throttle_percent * 1000.0f));

    return {{pulse_us, metrics}};
}
