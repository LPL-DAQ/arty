#include "HornetThrottle.h"
#include "../MutexGuard.h"
#include "../config.h"
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(hornet_throttle_lock);

/// Reset internal state before an active control trace
void HornetThrottle::reset()
{
    MutexGuard hornet_throttle_guard{&hornet_throttle_lock};
}

/// Generate a comomand for the main propeller.
std::expected<std::tuple<float, HornetThrottleMetrics>, Error> HornetThrottle::tick(float z_accel_m_s2)
{
    MutexGuard hornet_throttle_guard{&hornet_throttle_lock};
    HornetThrottleMetrics metrics = HornetThrottleMetrics_init_default;

    // Convert vertical acceleration to thrust in Newtons, then to lbf
    constexpr float mass_kg = 6.8f; // 15 lbs – TODO: get real mass estimate
    const float thrust_N = z_accel_m_s2 * mass_kg;
    const float thrust_command_lbf = thrust_N * N_TO_LBF;
    metrics.thrust_N = thrust_N;

    float best_fit_output;
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
