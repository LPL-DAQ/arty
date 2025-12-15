#ifndef CLOVER_SERVOTESTING_H
#define CLOVER_SERVOTESTING_H

#include <zephyr/drivers/pwm.h>
/* Simple arming/idle step for the ESCs (e.g., hold ~1000 µs). */
int servos_init();
/* Move both servos to  target degrees */
int servo_write_deg(const pwm_dt_spec& servo, float deg,
                    uint16_t min_us = 1000, uint16_t max_us = 2000);
/* Set raw throttle pulses for both ESCs in microseconds. */
int esc_write_us(const pwm_dt_spec& esc, uint16_t us);
/* Quick bring-up routine: arm ESCs and sweep servos. */
int servotesting_demo();
/* Convenience presets. */
void servo_neutral();  /* servos ~90°, ESCs neutral if used */
void esc_idle();     /* ESCs ~1000 µs, servos unchanged   */

#endif // CLOVER_SERVOTESTING_H

