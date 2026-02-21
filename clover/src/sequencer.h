#ifndef APP_SEQUENCER_H
#define APP_SEQUENCER_H

#include <stdint.h>
#include "clover.pb.h"

/**
 * @brief Loads sequences from a protobuf request.
 * Matches the call on Controller.cpp:74.
 */
void sequencer_load_from_proto(const LoadMotorSequenceRequest& req);

/**
 * @brief Clears current sequences and resets internal traces.
 */
void sequencer_reset();

/**
 * @brief Returns the total number of 1ms steps in the currently loaded sequence.
 * Matches the call on Controller.cpp:75.
 */
uint32_t sequencer_get_total_steps();

/**
 * @brief Samples the fuel valve position for a specific millisecond step.
 */
float sequencer_get_fuel_at(uint32_t step);

/**
 * @brief Samples the lox valve position for a specific millisecond step.
 */
float sequencer_get_lox_at(uint32_t step);

#endif // APP_SEQUENCER_H
