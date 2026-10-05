#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

/// Ranger TVC. A dedicated 50 Hz thread (RangerTvc.cpp) owns the CAN bus and both moteus controllers; everything here
/// is a non-blocking hand-off to or from that thread, safe to call from the controller workqueue every tick.
namespace RangerTvc {

using Telemetry = std::tuple<TvcActuatorCommand, TvcActuatorCommand, RangerTvcMetrics>;

enum class HomingStatus {
    IN_PROGRESS,
    SUCCEEDED,
    FAILED,
};

/// Bring up the CAN bus and start the TVC thread. Call once from main().
std::expected<void, Error> init();

/// Reset internal state before an active control trace. The TVC loop is continuous, so there is nothing to reset.
void reset();

/// Hand the latest gimbal command to the TVC loop and return its latest telemetry (pitch command, yaw command, metrics).
/// Fails, aborting active control, if the TVC is not enabled or has a latched fault.
std::expected<Telemetry, Error> tick(float pitch_command_deg, float yaw_command_deg);

/// Command the gimbal back to center (used by ABORT).
void hold_center();

/// Latest telemetry without changing the command.
Telemetry telemetry();

/// Whether the TVC is enabled and fault-free, i.e. ready for an active control sequence.
std::expected<void, Error> ensure_ready();

/// Start homing: the operator must have mechanically centered the gimbal. Poll homing_status() for the outcome.
void request_home();
HomingStatus homing_status();

}  // namespace RangerTvc
