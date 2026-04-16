#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

#include "sensors/AnalogSensors.h"
#include "Error.h"
#include "Trace.h"
#include "clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>

class Controller {
public:
    // Define nominal safe positions

    // Shared tracking variables
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;

    static inline SystemState current_state = SystemState_STATE_IDLE;

    static std::expected<void, Error> init();
    static void controller_step_control_loop(k_work* work);  // The 1ms dispatcher called by the timer
    static void control_loop_schedule(k_timer* timer);

    static void step_control_loop(k_work*);
    // Request handlers
    static std::expected<void, Error> handle_abort(const AbortRequest& req);
    static std::expected<void, Error> handle_halt(const HaltRequest& req);
    static std::expected<void, Error> handle_prime(const PrimeRequest& req);
    static std::expected<void, Error> handle_calibrate_throttle(const ThrottleCalibrateValveRequest& req);
    static std::expected<void, Error> handle_load_throttle_valve_sequence(const ThrottleLoadValveSequenceRequest& req);
    static std::expected<void, Error> handle_load_throttle_thrust_sequence(const ThrottleLoadThrustSequenceRequest& req);
    static std::expected<void, Error> handle_load_rcs_valve_sequence(const RCSLoadValveSequenceRequest& req);
    static std::expected<void, Error> handle_load_rcs_roll_sequence(const RCSLoadRollSequenceRequest& req);
    static std::expected<void, Error> handle_load_tvc_sequence(const TVCLoadSequenceRequest& req);
    static std::expected<void, Error> handle_start_throttle_valve_sequence(const ThrottleStartValveSequenceRequest& req);
    static std::expected<void, Error> handle_start_throttle_thrust_sequence(const ThrottleStartThrustSequenceRequest& req);
    static std::expected<void, Error> handle_start_rcs_valve_sequence(const RCSStartValveSequenceRequest& req);
    static std::expected<void, Error> handle_start_rcs_roll_sequence(const RCSStartRollSequenceRequest& req);
    static std::expected<void, Error> handle_start_tvc_sequence(const TVCStartSequenceRequest& req);
    static std::expected<void, Error> handle_load_flight_sequence(const FlightLoadSequenceRequest& req);
    static std::expected<void, Error> handle_start_flight_sequence(const FlightStartSequenceRequest& req);
    static const char* get_state_name(SystemState state);

    static bool should_tick_throttle();
    static bool should_tick_tvc();
    static bool should_tick_rcs();
    static bool should_tick_flight();
    static std::expected<void, Error> attempt_abort_subsystems();
    Controller() = delete;  // Explicitly prevent instantiation

private:
    static inline uint32_t udp_sequence_number = 0;
};

extern struct k_msgq telemetry_msgq;

#endif  // APP_CONTROLLER_H
