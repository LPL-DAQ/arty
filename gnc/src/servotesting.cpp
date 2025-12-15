#include "servotesting.h"

#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

#define USER_NODE DT_PATH(zephyr_user)

// Validate devicetree
#if !DT_NODE_HAS_PROP(USER_NODE, pwm_names)
#error "pts: Missing `pwm-names` property from `zephyr-user` node."
#endif

LOG_MODULE_REGISTER(servotesting, CONFIG_LOG_DEFAULT_LEVEL);

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
    if (!pwm_ready(ESC_1) || !pwm_ready(ESC_2) ||
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

/* ESC helper: write a raw microsecond pulse (e.g., 1000–2000 µs) */
int esc_write_us(const pwm_dt_spec& esc, uint16_t us) {
    if (!pwm_ready(esc)) return -ENODEV;
    /* Clamp to something sane for hobby ESCs */
    if (us < 800)  us = 800;
    if (us > 2200) us = 2200;
    return pwm_set_pulse_dt(&esc, PWM_USEC(us));
}

/* Simple demo: sweep servos and blip ESCs */
int servotesting_demo() {
    printk("Servotesting demo\n");
    int err = servos_init();
    if (err) return err;

    printk("arm esc\n");

    /* Example: arm ESCs (typical: 1000 µs for a couple seconds) */
    esc_write_us(ESC_1, 1000);
    esc_write_us(ESC_2, 1000);
    k_msleep(1000);

    printk("small throttle\n");
    printk("servo 0 -> 180 \n");
    /* servos 90 */
    while (true){

        for (int d = 0; d <= 180; d += 2) {

            int err = 0;
            err = servo_write_deg(SERVO_X, d);
            if (err) {
                LOG_ERR("Failed to write servo X");
                return 0;
            }

            err = servo_write_deg(SERVO_Y, d);
            if (err) {
                LOG_ERR("Failed to write servo Y");
                return 0;
            }
            esc_write_us(ESC_1, 1200);
            esc_write_us(ESC_2, 1200);
            k_msleep(20);
        }
    printk("servo 180 -> 0 \n");

        /* servos 90 */
        for (int d = 0; d <= 180; d += 2) {

            int err = 0;
            err = servo_write_deg(SERVO_X, 180-d);
            err = servo_write_deg(SERVO_Y, 180-d);
            if (err) {
                LOG_ERR("Failed to write servo X");
                return 0;
            }

            if (err) {
                LOG_ERR("Failed to write servo Y");
                return 0;
            }
            esc_write_us(ESC_1, 1000);
            esc_write_us(ESC_2, 1000);
            k_msleep(20);
        }
    }

    printk("demo over: idle all\n");

    /* Idle ESCs again */
    esc_idle();
    servo_neutral();
    return 0;
}

void servo_neutral() {
    servo_write_deg(SERVO_X, 90);
    servo_write_deg(SERVO_Y, 90);
}

void esc_idle() {
    esc_write_us(ESC_1, 1000);
    esc_write_us(ESC_2, 1000);

}