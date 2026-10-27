#include "HornetRcs.h"
#include "../MutexGuard.h"
#include "../PID.h"
#include "../config.h"
#include <algorithm>
#include <cmath>
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(hornet_rcs_lock);

static PID roll_pid(HORNET_RCS_ROLL_KP, HORNET_RCS_ROLL_KI, HORNET_RCS_ROLL_KD);
static constexpr float MIN_PULSE_S = 0.10f;
static constexpr float DEADZONE = 0.15f;
static constexpr float HYSTERESIS = 0.05f;

static bool has_previous_timestamp = false;
static int64_t previous_timestamp_us = 0;
static float p_timer_s = 0.0f;
static int8_t p_valve = 0;

#if CONFIG_TEST
static bool now_us_override_set = false;
static int64_t now_us_override = 0;
#endif

static int64_t current_time_us()
{
#if CONFIG_TEST
    if (now_us_override_set) {
        return now_us_override;
    }
#endif
    return static_cast<int64_t>(k_cyc_to_us_floor64(k_cycle_get_64()));
}

/// Seconds since the previous tick, or 0 on the first tick after reset.
static float tick_dt_s()
{
    const int64_t now_us = current_time_us();
    const float dt_s = has_previous_timestamp ? static_cast<float>(now_us - previous_timestamp_us) * 1e-6f : 0.0f;
    previous_timestamp_us = now_us;
    has_previous_timestamp = true;
    return dt_s;
}

static int control(EstimatedState state, float desired_roll_deg, HornetRcsMetrics& metrics)
{
    const float dt_s = tick_dt_s();

    Quaternion q_wb = state.R_WB;
    q_wb = math_util::normalizeQuaternion(q_wb);
    // yaw, pitch, roll
    Vector3D euler = math_util::quaternionToEulerAngles(q_wb);
    // Rotation about body x. Whether that is roll depends on which body axis is the thrust axis.
    const float roll_deg = euler.z * RAD2DEG_F;
    // TODO: itd be nice to have angular rates in the state estimate
    // float roll_velocity = 0.0;

    float control_effort = roll_pid.calculate(desired_roll_deg, roll_deg, dt_s);

    // if p_timer is active, don't change until it counts down
    if (p_timer_s > 0.0f) {
        p_timer_s = std::max(p_timer_s - dt_s, 0.0f);
    }
    // if closed
    else if (p_valve == 0) {
        if (control_effort > DEADZONE) {
            p_valve = 1;
            p_timer_s = MIN_PULSE_S;
        }
        else if (control_effort < -DEADZONE) {
            p_valve = -1;
            p_timer_s = MIN_PULSE_S;
        }
    }
    // if open
    else if (p_valve == 1) {
        if (control_effort < (DEADZONE - HYSTERESIS)) {
            p_valve = 0;
        }
    }
    // if open
    else if (p_valve == -1) {
        if (control_effort > -(DEADZONE - HYSTERESIS)) {
            p_valve = 0;
        }
    }

    metrics.measured_roll_deg = roll_deg;
    metrics.control_effort = control_effort;
    metrics.valve_state = p_valve;
    metrics.remaining_on_time_s = p_timer_s;
    return p_valve;
}

/// Reset internal state before an active control trace
void HornetRcs::reset()
{
    MutexGuard hornet_rcs_guard{&hornet_rcs_lock};
    roll_pid.reset();
    has_previous_timestamp = false;
    previous_timestamp_us = 0;
    p_timer_s = 0.0f;
    p_valve = 0;
#if CONFIG_TEST
    now_us_override_set = false;
#endif
}

uint32_t HornetRcs::bench_throttle_percent_to_pulse_us(float throttle_percent)
{
    const float percent = throttle_percent > 0.0f ? std::min(throttle_percent, HORNET_RCS_BENCH_MAX_THROTTLE_PERCENT) : 0.0f;
    return static_cast<uint32_t>(std::lround(MIN_PWM_PULSE_US + percent / 100.0f * (MAX_PWM_PULSE_US - MIN_PWM_PULSE_US)));
}

/// Generate a comomand for the cs and ccs RCS propellers.
std::expected<std::tuple<float, float, HornetRcsMetrics>, Error> HornetRcs::tick(EstimatedState state, float roll_command_deg)
{
    MutexGuard hornet_rcs_guard{&hornet_rcs_lock};
    HornetRcsMetrics metrics = HornetRcsMetrics_init_default;
    int control_var = control(state, roll_command_deg, metrics);

    const float cw_throttle  = control_var == -1  ? HORNET_RCS_THROTTLE_PERCENT : 0.0f;
    const float ccw_throttle = control_var == 1 ? HORNET_RCS_THROTTLE_PERCENT : 0.0f;
    const uint32_t cw_pulse_us  = static_cast<uint32_t>(std::lround(MIN_PWM_PULSE_US + (cw_throttle  * (MAX_PWM_PULSE_US - MIN_PWM_PULSE_US))));
    const uint32_t ccw_pulse_us = static_cast<uint32_t>(std::lround(MIN_PWM_PULSE_US + (ccw_throttle * (MAX_PWM_PULSE_US - MIN_PWM_PULSE_US))));

    return {{cw_pulse_us, ccw_pulse_us, metrics}};
}

#if CONFIG_TEST
void HornetRcs::set_now_us_for_testing(int64_t us)
{
    now_us_override = us;
    now_us_override_set = true;
}
#endif
