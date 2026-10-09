// Declares the controller initialization, telemetry, and command-handler interface.
#pragma once

#include "Error.h"
#include "Trace.h"
#include "clover.pb.h"
#include "sensors/AnalogSensors.h"
#include <expected>
#include <zephyr/kernel.h>

namespace Controller {
constexpr float ABORT_TIME_MSEC = 500;
constexpr uint64_t NSEC_PER_CONTROL_TICK = 1'500'000;  // 1.5 ms
constexpr float SEC_PER_CONTROL_TICK = NSEC_PER_CONTROL_TICK * 1e-9f;

/// Initializes controller timing, work queues, and dependent subsystems.
/// Parameters: None.
/// Returns: success or an initialization error.
std::expected<void, Error> init();

/// Retrieves the next telemetry packet from the controller queue.
/// Parameters: None.
/// Returns: the next queued packet.
DataPacket get_next_data_packet();

/// Resets a Ranger throttle valve's reported position.
/// Parameters: req identifies the valve and requested position.
/// Returns: success or a configuration/actuator error.
std::expected<void, Error> handle_throttle_reset_valve_position(const ThrottleResetValvePositionRequest& req);

/// Requests an abort state transition.
/// Parameters: req is the abort request.
/// Returns: success or an error if safe-state restoration fails.
std::expected<void, Error> handle_abort(const AbortRequest& req);

/// Halts active control and returns the controller to IDLE.
/// Parameters: req is the halt request.
/// Returns: success or an error if no sequence is active or restoration fails.
std::expected<void, Error> handle_halt(const HaltRequest& req);

/// Cancels a primed sequence and returns the controller to IDLE.
/// Parameters: req is the unprime request.
/// Returns: success or an error when the controller is not primed.
std::expected<void, Error> handle_unprime(const UnprimeRequest& req);

/// Starts an autonomous valve sequence from IDLE.
/// Parameters: req identifies the sequence, run, and completion valve behavior.
/// Returns: success or a parsing, state, valve, or scheduler error.
std::expected<void, Error> handle_run_autonomous_valve_sequence(const RunAutonomousValveSequenceRequest& req);

/// Accepts one autonomous valve sequence upload chunk.
/// Parameters: req contains the file, chunk, ordering, and run-selection data.
/// Returns: success or an upload/state/validation error.
std::expected<void, Error> handle_upload_autonomous_valve_sequence_chunk(const UploadAutonomousValveSequenceChunkRequest& req);

/// Starts calibration of a throttle valve.
/// Parameters: req identifies the valve to calibrate.
/// Returns: success or a state/configuration/calibration error.
std::expected<void, Error> handle_calibrate_throttle_valve(const CalibrateThrottleValveRequest& req);

/// Loads throttle-valve traces and primes the controller.
/// Parameters: req contains the fuel and oxidizer traces.
/// Returns: success or a state/trace error.
std::expected<void, Error> handle_load_throttle_valve_sequence(const LoadThrottleValveSequenceRequest& req);

/// Starts a previously primed throttle-valve sequence.
/// Parameters: req is the start request.
/// Returns: success or an error if the controller is not primed.
std::expected<void, Error> handle_start_throttle_valve_sequence(const StartThrottleValveSequenceRequest& req);

/// Loads the thrust trace and primes the throttle controller.
/// Parameters: req contains the thrust trace.
/// Returns: success or a state/trace error.
std::expected<void, Error> handle_load_throttle_sequence(const LoadThrottleSequenceRequest& req);

/// Starts a previously primed throttle sequence.
/// Parameters: req is the start request.
/// Returns: success or an error if the controller is not primed.
std::expected<void, Error> handle_start_throttle_sequence(const StartThrottleSequenceRequest& req);

/// Starts TVC calibration.
/// Parameters: req is the calibration request.
/// Returns: success or a state/configuration/calibration error.
std::expected<void, Error> handle_calibrate_tvc(const CalibrateTvcRequest& req);

/// Loads pitch and yaw traces and primes the TVC controller.
/// Parameters: req contains the pitch and yaw traces.
/// Returns: success or a state/trace error.
std::expected<void, Error> handle_load_tvc_sequence(const LoadTvcSequenceRequest& req);

/// Starts a previously primed TVC sequence.
/// Parameters: req is the start request.
/// Returns: success or an error if the controller is not primed.
std::expected<void, Error> handle_start_tvc_sequence(const StartTvcSequenceRequest& req);

/// Loads clockwise and counterclockwise RCS-valve traces and primes the controller.
/// Parameters: req contains the two valve traces.
/// Returns: success or a state/trace error.
std::expected<void, Error> handle_load_rcs_valve_sequence(const LoadRcsValveSequenceRequest& req);

/// Starts a previously primed RCS-valve sequence.
/// Parameters: req is the start request.
/// Returns: success or an error if the controller is not primed.
std::expected<void, Error> handle_start_rcs_valve_sequence(const StartRcsValveSequenceRequest& req);

/// Loads the RCS roll trace and primes the RCS controller.
/// Parameters: req contains the roll trace.
/// Returns: success or a state/trace error.
std::expected<void, Error> handle_load_rcs_sequence(const LoadRcsSequenceRequest& req);

/// Starts a previously primed RCS sequence.
/// Parameters: req is the start request.
/// Returns: success or an error if the controller is not primed.
std::expected<void, Error> handle_start_rcs_sequence(const StartRcsSequenceRequest& req);

/// Loads static-fire thrust and TVC traces and primes the controller.
/// Parameters: req contains all required static-fire traces.
/// Returns: success or a state/trace error.
std::expected<void, Error> handle_load_static_fire_sequence(const LoadStaticFireSequenceRequest& req);

/// Starts a previously primed static-fire sequence.
/// Parameters: req is the start request.
/// Returns: success or an error if the controller is not primed.
std::expected<void, Error> handle_start_static_fire_sequence(const StartStaticFireSequenceRequest& req);

/// Loads flight position and attitude traces and primes the flight controller.
/// Parameters: req contains the flight reference traces.
/// Returns: success or a state/trace error.
std::expected<void, Error> handle_load_flight_sequence(const LoadFlightSequenceRequest& req);

/// Starts a previously primed flight sequence.
/// Parameters: req is the start request.
/// Returns: success or an error if the controller is not primed.
std::expected<void, Error> handle_start_flight_sequence(const StartFlightSequenceRequest& req);
}  // namespace Controller
