#ifndef CLOVER_ENCODER_H
#define CLOVER_ENCODER_H

#include <cstdint>

// Initialize quadrature encoder GPIO + interrupts
// Returns 0 on success, negative error code on failure
int encoder_init();

// Get current encoder position in ticks (signed, relative to boot)
int32_t encoder_get_position();

// Get corresponding output-shaft angle in degrees
float encoder_get_degrees();

#endif // CLOVER_ENCODER_H
