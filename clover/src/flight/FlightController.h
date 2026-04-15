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

namespace FlightController {

    static inline FlightState current_state = FlightState_FLIGHT_STATE_IDLE;
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;

    FlightState state();

    void resetPIDs();


    std::expected<void, Error> init();
    void step_control_loop(DataPacket& data);

    std::expected<void, Error> load_sequence(const FlightLoadSequenceRequest& req);
    std::expected<void, Error> start_sequence();


    std::expected<void, Error> change_state(FlightState new_state);
    const char* get_state_name(FlightState state);

    std::pair<FlightStateOutput, FlightIdleData> idle_tick();
    std::pair<FlightStateOutput, FlightTakeoffData> takeoff_tick(int64_t current_time, int64_t start_time);
    std::pair<FlightStateOutput, FlightSequenceData> flight_seq_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time);
    std::pair<FlightStateOutput, FlightLandingData> landing_tick(int64_t current_time, int64_t start_time);
    std::pair<FlightStateOutput, FlightAbortData> abort_tick(int64_t current_time, int64_t entry_time);

    static inline FlightStateOutput current_output = FlightStateOutput_init_default;
}

#endif  // APP_FLIGHT_CONTROLLER_H
