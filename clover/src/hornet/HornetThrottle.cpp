#include "HornetThrottle.h"
#include "../MutexGuard.h"
#include "../config.h"
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
    HornetThrottleMetrics metrics = HornetThrottleMetrics_init_default;
    float best_fit_output;

    // TODO: fix. N
    float throttle_percent;
    if (thrust_command_lbf < HORNET_THROTTLE_LOW_THRUST_THRESHOLD_LBF){ // below 10 lbf (only ever during landing and takeoff sequences)s)
        best_fit_output = (thrust_command_lbf / HORNET_THROTTLE_LOW_SLOPE);
        throttle_percent = std::clamp(best_fit_output, 0.0f, HORNET_THROTTLE_CROSSOVER_PERCENT);

    } else { // the actual operating range
        best_fit_output = (thrust_command_lbf + HORNET_THROTTLE_HIGH_OFFSET) / HORNET_THROTTLE_HIGH_SLOPE;
        throttle_percent = std::clamp(best_fit_output, HORNET_THROTTLE_CROSSOVER_PERCENT, 1.0f);
    }


    // Convert throttle to pulse width: 0% -> MIN_PWM_PULSE_US, 100% -> MAX_PWM_PULSE_US
    const uint32_t pulse_us = static_cast<uint32_t>(MIN_PWM_PULSE_US + (throttle_percent * (MAX_PWM_PULSE_US - MIN_PWM_PULSE_US)));

    return {{pulse_us, metrics}};
}
