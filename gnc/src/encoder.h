#ifndef CLOVER_ENCODER_H
#define CLOVER_ENCODER_H

#include <cstdint>

/**
    * Initialize the quadrature encoder GPIO/interrupts
    * Returns 0 on success ... negative error code on fail
*/
int encoder_init();

/**
    Get current encoder position in counts [relative/signed]
*/
int32_t encoder_get_position();

/**
    Get raw "velocity" quantity [counts per last measurement interval, 0 if idle]
*/
int32_t encoder_get_velocity_raw();

#endif // CLOVER_ENCODER_H
