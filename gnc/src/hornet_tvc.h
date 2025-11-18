#ifndef CLOVER_HORNET_TVC_H
#define CLOVER_HORNET_TVC_H

#include "util/math.h"
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
/* Simple arming/idle step for the ESCs (e.g., hold ~1000 µs). */
int servos_init();

int esc_init();

/* Move both servos to achieve desired vector angle. */
int set_desired_thrust_deg(double thrust_deg_x, double thrust_deg_y);

/* Move both servos to target degrees */
int servo_write_deg(const pwm_dt_spec& servo, float deg,
                    uint16_t min_us = 1000, uint16_t max_us = 2000);

/* Set ESC's to achieve desired thrust in newtons. */
int set_desired_thrust_newtons(double thrust_a_n, double thrust_b_n, double max_thrust_n = 2.5);

/* Set raw throttle pulses for both ESCs in microseconds. */
int esc_write_us(const pwm_dt_spec& esc, uint16_t us);
/* Quick bring-up routine: arm ESCs and sweep servos. */

int servo_neutral();  /* servos ~90°, ESCs neutral if used */
int esc_idle();     /* ESCs ~1000 µs, servos unchanged   */

#endif //CLOVER_HORNET_TVC_H

