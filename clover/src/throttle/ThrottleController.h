#ifndef APP_THROTTLE_CONTROLLER_H
#define APP_THROTTLE_CONTROLLER_H

#include "../sensors/AnalogSensors.h"
#include "../Error.h"
#include "../Trace.h"
#include "clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>

typedef ThrottleState ThrottleState;


class ThrottleController {
public:
    // Define nominal safe positions
    static constexpr float DEFAULT_FUEL_POS = 81.0f;
    static constexpr float DEFAULT_LOX_POS = 74.0f;
    // Shared tracking variables
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;
    static inline bool fuel_powered = true;
    static inline bool lox_powered = true;

    static inline ThrottleState current_state = ThrottleState_THROTTLE_STATE_IDLE;
    static ThrottleState state()
    {
        return current_state;
    }

    static std::expected<void, Error> init();

    static void step_control_loop(DataPacket& data);

    // Request handlers
    static std::expected<void, Error> handle_abort(const AbortRequest& req);
    static std::expected<void, Error> handle_load_valve_sequence(const ThrottleLoadValveSequenceRequest& req);
    static std::expected<void, Error> handle_start_valve_sequence(const ThrottleStartValveSequenceRequest& req);
    static std::expected<void, Error> handle_load_thrust_sequence(const ThrottleLoadThrustSequenceRequest& req);
    static std::expected<void, Error> handle_start_thrust_sequence(const ThrottleStartThrustSequenceRequest& req);
    static std::expected<void, Error> handle_unprime(const ThrottleUnprimeRequest& req);
    static std::expected<void, Error> handle_calibrate_valve(const ThrottleCalibrateValveRequest& req);

    static std::expected<void, Error> handle_halt(const ThrottleHaltRequest& req);

    static std::expected<void, Error> handle_reset_valve_position(const ThrottleResetValvePositionRequest& req);
    static std::expected<void, Error> handle_power_on(const ThrottlePowerOnRequest& req);
    static std::expected<void, Error> handle_power_off(const ThrottlePowerOffRequest& req);

    static std::expected<void, Error> change_state(ThrottleState new_state);
    static const char* get_state_name(ThrottleState state);
    ThrottleController() = delete;  // Explicitly prevent instantiation

private:
    static std::pair<ThrottleStateOutput, ThrottleIdleData> idle_tick();
    static std::pair<ThrottleStateOutput, ThrottleValveCalibrationData> calibrate_valve_tick(uint32_t current_time);
    static std::pair<ThrottleStateOutput, ThrottleIdleData> valve_primed_tick();
    static std::pair<ThrottleStateOutput, ThrottleValveSequenceData> valve_sequence_tick(int64_t current_time);
    static std::pair<ThrottleStateOutput, ThrottleIdleData> thrust_primed_tick();
    static std::pair<ThrottleStateOutput, ThrottleThrustSequenceData> thrust_sequence_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time);
    static std::pair<ThrottleStateOutput, ThrottleFlightData> flight_tick(const AnalogSensorReadings& analog_sensors);
    static std::pair<ThrottleStateOutput, ThrottleAbortData> abort_tick(uint32_t current_time);

    static inline bool valve_sequence_has_fuel = false;
    static inline bool valve_sequence_has_lox = false;
    static inline float valve_sequence_fuel_total_time_ms = -1.0f;
    static inline float valve_sequence_lox_total_time_ms = -1.0f;
    static inline float thrust_sequence_total_time_ms = 0.0f;
    static inline uint32_t udp_sequence_number = 0;
};

extern struct k_msgq telemetry_msgq;

#endif  // APP_THROTTLE_CONTROLLER_H
