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
    static void step_control_loop(DataPacket& data, std::optional<std::pair<AnalogSensorReadings, float>> analog_sensors_readings);

    static const FlightControllerOutput& get_output();
    static float get_z_acceleration();
    static float get_x_angular_acceleration();
    static float get_y_angular_acceleration();
    static float get_roll_position();

    static std::expected<void, Error> change_state(FlightState new_state);
    static const char* get_state_name(FlightState state);

    FlightController() = delete;  // Explicitly prevent instantiation

private:
    static inline FlightControllerOutput current_output = FlightControllerOutput_init_default;
};

#endif  // APP_FLIGHT_CONTROLLER_H
