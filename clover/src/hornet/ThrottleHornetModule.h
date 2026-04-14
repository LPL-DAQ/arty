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

static inline uint32_t abort_entry_time = 0;
static inline uint32_t sequence_start_time = 0;

static inline ThrottleState current_state = ThrottleState_THROTTLE_STATE_IDLE;
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

static inline float thrust_sequence_total_time_ms = 0.0f;

static Trace throttle_thrust_trace;
static Trace fuel_trace;
static Trace lox_trace;

// TODO: is this used anywhere?
static inline uint32_t udp_sequence_number = 0;

} // namespace ThrottleHornetModule

extern struct k_msgq telemetry_msgq;

#endif  // APP_THROTTLE_HORNET_MODULE_H
