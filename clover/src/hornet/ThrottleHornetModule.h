#ifndef APP_THROTTLE_HORNET_MODULE_H
#define APP_THROTTLE_HORNET_MODULE_H

#include "../sensors/AnalogSensors.h"
#include "../Error.h"
#include "../Trace.h"
#include "clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>

typedef ThrottleState ThrottleState;


namespace ThrottleHornetModule {

ThrottleState state();

void step_control_loop(DataPacket& data);

// Request handlers
std::expected<void, Error> load_thrust_sequence(const ThrottleLoadThrustSequenceRequest& req);
std::expected<void, Error> start_thrust_sequence();

std::expected<void, Error> change_state(ThrottleState new_state);
const char* get_state_name(ThrottleState state);

std::pair<ThrottleHornetStateOutput, ThrottleIdleData> idle_tick();
std::pair<ThrottleHornetStateOutput, ThrottleHornetThrustSequenceData> thrust_sequence_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time);
std::pair<ThrottleHornetStateOutput, ThrottleFlightData> flight_tick(const AnalogSensorReadings& analog_sensors, FlightStateOutput& flight_output);
std::pair<ThrottleHornetStateOutput, ThrottleAbortData> abort_tick(uint32_t current_time, uint32_t entry_time);




} // namespace ThrottleHornetModule

extern struct k_msgq telemetry_msgq;

#endif  // APP_THROTTLE_HORNET_MODULE_H
