#include "HornetRcs.h"
#include "../MutexGuard.h"
#include "../PID.h"
#include "../config.h"
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(hornet_rcs_lock);

static PID roll_pid(HORNET_RCS_ROLL_KP, HORNET_RCS_ROLL_KI, HORNET_RCS_ROLL_KD);
static int64_t previous_timestamp = 0;
static int64_t p_timer = 0;
static int8_t p_valve = 0;
static float min_pulse = 0.10;
static float deadzone = 0.15;
static float hysteresis = 0.05;

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

    // if p_timer is active, don't change until it counts down
    if (p_timer > 0){
        p_timer -= dt;
        if (p_timer < 0){
            p_timer = 0;
        }
    }
    // if closed
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
    // if open
    else if (p_valve == 1) {
        if (control_effort < (deadzone - hysteresis)){
            p_valve = 0;
        }
    }
    // if open
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
std::expected<std::tuple<float, float, HornetRcsMetrics>, Error> HornetRcs::tick(EstimatedState state, float roll_command_deg)
{
    MutexGuard hornet_rcs_guard{&hornet_rcs_lock};
    HornetRcsMetrics metrics = HornetRcsMetrics_init_default;
    int control_var = control(state, roll_command_deg);

    const float cw_throttle  = control_var == -1  ? HORNET_RCS_THROTTLE_PERCENT : 0.0f;
    const float ccw_throttle = control_var == 1 ? HORNET_RCS_THROTTLE_PERCENT : 0.0f;
    const uint32_t cw_pulse_us  = static_cast<uint32_t>(MIN_PWM_PULSE_US + (cw_throttle  * (MAX_PWM_PULSE_US - MIN_PWM_PULSE_US)));
    const uint32_t ccw_pulse_us = static_cast<uint32_t>(MIN_PWM_PULSE_US + (ccw_throttle * (MAX_PWM_PULSE_US - MIN_PWM_PULSE_US)));

    return {{cw_pulse_us, ccw_pulse_us, metrics}};
}
