#pragma once

#include "Error.h"
#include "Trace.h"
#include "clover.pb.h"
#include "sensors/AnalogSensors.h"
#include <expected>
#include <zephyr/kernel.h>

namespace Controller {
constexpr float ABORT_TIME_MSEC = 500;
constexpr uint64_t NSEC_PER_CONTROL_TICK = 1'000'000; // 1 ms
constexpr float SEC_PER_CONTROL_TICK = NSEC_PER_CONTROL_TICK * 1e-9f;

std::expected<void, Error> init();
DataPacket get_next_data_packet();

// Request handlers
std::expected<void, Error> handle_abort(const AbortRequest& req);
std::expected<void, Error> handle_halt(const HaltRequest& req);
std::expected<void, Error> handle_unprime(const UnprimeRequest& req);

// Throttle
std::expected<void, Error> handle_calibrate_throttle_valve(const CalibrateThrottleValveRequest& req);

std::expected<void, Error> handle_load_throttle_valve_sequence(const LoadThrottleValveSequenceRequest& req);
std::expected<void, Error> handle_start_throttle_valve_sequence(const StartThrottleValveSequenceRequest& req);

std::expected<void, Error> handle_load_throttle_sequence(const LoadThrottleSequenceRequest& req);
std::expected<void, Error> handle_start_throttle_sequence(const StartThrottleSequenceRequest& req);

// TVC
std::expected<void, Error> handle_calibrate_tvc(const CalibrateTvcRequest& req);

std::expected<void, Error> handle_load_tvc_sequence(const LoadTvcSequenceRequest& req);
std::expected<void, Error> handle_start_tvc_sequence(const StartTvcSequenceRequest& req);

// RCS
std::expected<void, Error> handle_load_rcs_valve_sequence(const LoadRcsValveSequenceRequest& req);
std::expected<void, Error> handle_start_rcs_valve_sequence(const StartRcsValveSequenceRequest& req);

std::expected<void, Error> handle_load_rcs_sequence(const LoadRcsSequenceRequest& req);
std::expected<void, Error> handle_start_rcs_sequence(const StartRcsSequenceRequest& req);

// Static flight (TVC + Throttle)
std::expected<void, Error> handle_load_static_fire_sequence(const LoadStaticFireSequenceRequest& req);
std::expected<void, Error> handle_start_static_fire_sequence(const StartStaticFireSequenceRequest& req);

// Flight
std::expected<void, Error> handle_load_flight_sequence(const LoadFlightSequenceRequest& req);
std::expected<void, Error> handle_start_flight_sequence(const StartFlightSequenceRequest& req);
};  // namespace Controller
