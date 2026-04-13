#ifndef APP_FLIGHT_CONTROLLER_H
#define APP_FLIGHT_CONTROLLER_H

#include "sensors/AnalogSensors.h"
#include "Error.h"
#include "Trace.h"
#include "clover.pb.h"
#include <expected>
#include <optional>
#include <utility>
#include <zephyr/kernel.h>

class FlightController {
public:
    static inline FlightState current_state = FlightState_FLIGHT_STATE_IDLE;
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;

    static std::expected<void, Error> init();
    static void step_control_loop(DataPacket& data);

    static std::expected<void, Error> handle_load_sequence(const FlightLoadSequenceRequest& req);
    static std::expected<void, Error> handle_start_sequence(const FlightStartSequenceRequest& req);
    static std::expected<void, Error> handle_halt(const FlightHaltRequest& req);

    static const FlightStateOutput& get_output();
    static float get_z_acceleration();
    static float get_x_angular_acceleration();
    static float get_y_angular_acceleration();
    static float get_roll_position();

    static std::expected<void, Error> change_state(FlightState new_state);
    static const char* get_state_name(FlightState state);

    FlightController() = delete;  // Explicitly prevent instantiation

private:
    static std::pair<FlightStateOutput, FlightIdleData> idle_tick();
    static std::pair<FlightStateOutput, FlightTakeoffData> takeoff_tick(int64_t current_time, int64_t start_time);
    static std::pair<FlightStateOutput, FlightSequenceData> flight_seq_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time);
    static std::pair<FlightStateOutput, FlightLandingData> landing_tick(int64_t current_time, int64_t start_time);
    static std::pair<FlightStateOutput, FlightAbortData> abort_tick(int64_t current_time, int64_t entry_time);

    static inline FlightStateOutput current_output = FlightStateOutput_init_default;
};

#endif  // APP_FLIGHT_CONTROLLER_H
