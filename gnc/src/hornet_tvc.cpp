#include "hornet_tvc.h"
#include <util/math.h>

#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

LOG_MODULE_REGISTER(hornet_tvc, CONFIG_LOG_DEFAULT_LEVEL);


// start with 1000-2000 us PWM signal, test to go up to 500-2500
static const pwm_dt_spec SERVO_X =
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), tvc_x);
static const pwm_dt_spec SERVO_Y =
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), tvc_y);

static const pwm_dt_spec ESC_1 =
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), esc_1);
static const pwm_dt_spec ESC_2 =
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), esc_2);



static inline bool pwm_ready(const pwm_dt_spec& s) {
    return device_is_ready(s.dev);
}

/* Initialize: verify all PWM endpoints are present & ready */
int servos_init() {
    if (
        !pwm_ready(SERVO_X) || !pwm_ready(SERVO_Y)) {
        return -ENODEV;
    }
    /* Optional: park outputs at neutral pulses if your PCA9685 default is 0 */
    pwm_set_pulse_dt(&SERVO_X, PWM_USEC(1500));
    pwm_set_pulse_dt(&SERVO_Y, PWM_USEC(1500));
    pwm_set_pulse_dt(&ESC_1,  PWM_USEC(1000));
    pwm_set_pulse_dt(&ESC_2,  PWM_USEC(1000));
    return 0;
}

int esc_init() {
    if (
        !pwm_ready(ESC_1) || !pwm_ready(ESC_2) ){
        return -ENODEV;
    }
    pwm_set_pulse_dt(&ESC_1,  PWM_USEC(1000));
    pwm_set_pulse_dt(&ESC_2,  PWM_USEC(1000));
    return 0;
}

// To be set later with 4-bar geometry and imu transformations, look at old drone code for reference
int set_desired_thrust_deg(double thrust_deg_x, double thrust_deg_y) {

      return 0;
}

/* Generic servo: degrees → microseconds → pwm_set_pulse_dt
   deg in [0,180] maps to [min_us,max_us] (defaults 1000–2000 µs) */
int servo_write_deg(const pwm_dt_spec& servo,
                    float deg,
                    uint16_t min_us,
                    uint16_t max_us)
{
    if (!pwm_ready(servo)) return -ENODEV;
    if (deg < 0.f)   deg = 0.f;
    if (deg > 180.f) deg = 180.f;
    const float span = float(max_us - min_us);
    const uint32_t us = uint32_t(min_us + (deg / 180.f) * span);
    return pwm_set_pulse_dt(&servo, PWM_USEC(us));
}

int set_desired_thrust_newtons(double thrust_a_n, double thrust_b_n, double max_thrust_n){
    // 1000-2000 us PWM signal, double check range later
    double esc_1_us = util::mapFloat(thrust_a_n, 0, max_thrust_n, 1000, 2000);
    esc_write_us(ESC_1, esc_1_us);
    double esc_2_us = util::mapFloat(thrust_b_n, 0, max_thrust_n, 1000, 2000);

    esc_write_us(ESC_2, esc_2_us);

}

/* ESC helper: write a raw microsecond pulse (e.g., 1000–2000 µs) */
int esc_write_us(const pwm_dt_spec& esc, uint16_t us) {
    if (!pwm_ready(esc)) return -ENODEV;
    /* Clamp to something sane for hobby ESCs */
    if (us < 800)  us = 800;
    if (us > 2200) us = 2200;
    return pwm_set_pulse_dt(&esc, PWM_USEC(us));
}

int servo_neutral() {
    servo_write_deg(SERVO_X, 90);
    return servo_write_deg(SERVO_Y, 90);
}

int esc_idle() {
    esc_write_us(ESC_1, 1000);
    return esc_write_us(ESC_2, 1000);

}