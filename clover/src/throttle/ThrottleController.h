#ifndef APP_THROTTLE_CONTROLLER_H
#define APP_THROTTLE_CONTROLLER_H

#include "../sensors/AnalogSensors.h"
#include "../Error.h"
#include "../Trace.h"
#include "../clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>

typedef SystemState SystemState;

struct ThrottleControllerOutput {
    bool set_fuel = false;
    float fuel_pos = 0.0f;
    bool fuel_on = true;
    bool set_lox = false;
    float lox_pos = 0.0f;
    bool lox_on = true;
    float reset_fuel_pos = 0.0f;
    bool reset_fuel = false;
    float reset_lox_pos = 0.0f;
    bool reset_lox = false;
    SystemState next_state = SystemState_STATE_IDLE;
};

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

    static inline SystemState current_state = SystemState_STATE_IDLE;
    static SystemState state()
    {
        return current_state;
    }

    static std::expected<void, Error> init();

    static void step_control_loop(std::optional<std::pair<AnalogSensorReadings, float>> analog_sensors_readings);
    // Request handlers
    static std::expected<void, Error> handle_load_valve_sequence(const LoadValveSequenceRequest& req);
    static std::expected<void, Error> handle_start_valve_sequence(const StartValveSequenceRequest& req);
    static std::expected<void, Error> handle_load_thrust_sequence(const LoadThrustSequenceRequest& req);
    static std::expected<void, Error> handle_start_thrust_sequence(const StartThrustSequenceRequest& req);
    static std::expected<void, Error> handle_abort(const AbortRequest& req);
    static std::expected<void, Error> handle_unprime(const UnprimeRequest& req);
    static std::expected<void, Error> handle_calibrate_valve(const CalibrateValveRequest& req);

    static std::expected<void, Error> handle_halt(const HaltRequest& req);

    static std::expected<void, Error> handle_reset_valve_position(const ResetValvePositionRequest& req);
    static std::expected<void, Error> handle_power_on_valve(const PowerOnValveRequest& req);
    static std::expected<void, Error> handle_power_off_valve(const PowerOffValveRequest& req);

    static std::expected<void, Error> change_state(SystemState new_state);
    static const char* get_state_name(SystemState state);
    ThrottleController() = delete;  // Explicitly prevent instantiation

private:
    static inline uint32_t udp_sequence_number = 0;
};

extern struct k_msgq telemetry_msgq;

#endif  // APP_THROTTLE_CONTROLLER_H
