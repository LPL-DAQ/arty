#include "HornetRcs.h"
#include "MutexGuard.h"
#include "PID.h"
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(hornet_rcs_lock);

namespace {
    PID roll_pid(2, 0, 5);
    int64_t previous_timestamp = 0;
    int64_t p_timer = 0;
    int8_t p_valve = 0;
    float min_pulse = 0.10;
    float deadzone = 0.15;
    float hysteresis = 0.05;

}

static int control(EstimatedState state, float desired_roll_position){
    int64_t dt = k_uptime_get() - previous_timestamp;

    Quaternion q_wb = state.R_WB;
    q_wb = math_util::normalizeQuaternion(q_wb);
    // yaw, pitch, roll
    Vector3D euler = math_util::quaternionToEulerAngles(q_wb);
    double roll_position = euler.z;
    // TODO: itd be nice to have angular rates in the state estimate
    // float roll_velocity = 0.0;

    float control_effort = roll_pid.calculate(desired_roll_position, roll_position, dt);

    if (p_timer > 0){
        p_timer -= dt;
        if (p_timer < 0){
            p_timer = 0;
        }
    }

    else if (p_valve == 0){
        if (control_effort > deadzone){
            p_valve = 1;
            p_timer = min_pulse;
        }
        else if (control_effort < -deadzone) {
            p_valve = -1;
            p_timer = min_pulse;
        }
    }
    else if (p_valve == 1) {
        if (control_effort < (deadzone - hysteresis)){
            p_valve = 0;
        }
    }

    else if (p_valve == -1) {
        if (control_effort > -(deadzone - hysteresis)){
            p_valve = 0;
        }
    }

    return p_valve;
}

/// Reset internal state before an active control trace
void HornetRcs::reset()
{
    MutexGuard hornet_rcs_guard{&hornet_rcs_lock};
    roll_pid.reset();
    previous_timestamp = 0;
    p_timer = 0;
    p_valve = 0;
    min_pulse = 0.10;
    deadzone = 0.15;
    hysteresis = 0.05;
}

/// Generate a comomand for the cs and ccs RCS propellers.
std::expected<std::tuple<bool, bool, HornetRcsMetrics>, Error> HornetRcs::tick(EstimatedState state, float roll_command_deg)
{
    MutexGuard hornet_rcs_guard{&hornet_rcs_lock};
    HornetRcsMetrics metrics = HornetRcsMetrics_init_default;
    int control_var = control(state, roll_command_deg);

    const float throttle_value = 1.0f;
    const float cw_throttle  = control_var == -1  ? throttle_value : 0.0f;
    const float ccw_throttle = control_var == 1 ? throttle_value : 0.0f;
    // Convert throttle to pulse width: 0.0 -> 1000µs, 1.0 -> 2000µs
    const uint32_t cw_pulse_us = static_cast<uint32_t>(1000.0f + (cw_throttle * 1000.0f));
    const uint32_t ccw_pulse_us = static_cast<uint32_t>(1000.0f + (ccw_throttle * 1000.0f));

    return {{cw_pulse_us, ccw_pulse_us, metrics}};
}
