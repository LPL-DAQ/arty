#ifndef APP_RCS_RANGER_MODULE_H
#define APP_RCS_RANGER_MODULE_H

#include "../sensors/AnalogSensors.h"
#include "../Error.h"
#include "../Trace.h"
#include "clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>

typedef RCSState RCSState;

namespace RCSRangerModule {
    RCSState state();

    void step_control_loop(DataPacket& data);
    // Request handlers
    std::expected<void, Error> load_motor_sequence(const RCSLoadValveSequenceRequest& req);
    std::expected<void, Error> load_roll_sequence(const RCSLoadRollSequenceRequest& req);
    std::expected<void, Error> start_motor_sequence();
    std::expected<void, Error> start_roll_sequence();

    std::expected<void, Error> change_state(RCSState new_state);
    const char* get_state_name(RCSState state);

    std::pair<RCSRangerStateOutput, RCSIdleData> idle_tick();
    std::pair<RCSRangerStateOutput, RCSValveSequenceData> motor_sequence_tick(int64_t current_time, int64_t start_time);
    std::pair<RCSRangerStateOutput, RCSRollSequenceData> roll_sequence_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time);
    std::pair<RCSRangerStateOutput, RCSFlightData> flight_tick(const AnalogSensorReadings& analog_sensors, FlightStateOutput& flight_output);
    std::pair<RCSRangerStateOutput, RCSAbortData> abort_tick(uint32_t current_time, uint32_t entry_time);

} // namespace RCSRangerModule

extern struct k_msgq telemetry_msgq;

#endif  // APP_RCS_RANGER_MODULE_H
