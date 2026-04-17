#include "PwmActuator.h"

LOG_MODULE_REGISTER(pwm_actuator, LOG_LEVEL_DBG);

const pwm_dt_spec servo_l_pwm     = PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), servo_l);
const pwm_dt_spec servo_r_pwm     = PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), servo_r);
const pwm_dt_spec motor_beta1_pwm = PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), motor_beta1);
const pwm_dt_spec motor_beta2_pwm = PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), motor_beta2);
const pwm_dt_spec motor_beta3_pwm = PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), motor_beta3);
const pwm_dt_spec motor_beta4_pwm = PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), motor_beta4);
const pwm_dt_spec motor_cr_pwm    = PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), motor_cr);