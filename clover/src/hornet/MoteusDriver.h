#pragma once

#include "../Error.h"
#include "moteus/moteus.h"
#include <expected>

namespace MoteusDriver {

// Controller::Result holds the raw CAN frame plus a Query::Result in .values.
using Reply = mjbots::moteus::Controller::Result;

// Call once at startup — creates the Zephyr CAN transport and constructs
// a Controller for each motor ID that will be commanded.
std::expected<void, Error> init();

// Stop the motor immediately (mode = Stopped).
std::expected<void, Error> stop(uint8_t motor_id);

// Command position (revolutions) with optional velocity feedforward (rev/s)
// and torque limit (N·m).  position = NaN → velocity-only mode.
// Returns the telemetry reply from the motor, or an empty optional on timeout.
std::expected<mjbots::moteus::Optional<Reply>, Error>
set_position(uint8_t motor_id,
             double position_rev,
             double velocity_rev_s = 0.0,
             double max_torque_Nm  = 0.0);

}  // namespace MoteusDriver
