#include "ThrottleHornetModule.h"
#include "../sensors/AnalogSensors.h"
#include "../HornetModuleConfig.h"
#include "../server.h"

#include "../config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#include <utility>


LOG_MODULE_REGISTER(ThrottleHornetModule, LOG_LEVEL_INF);



std::expected<void, Error> ThrottleHornetModule::change_state(ThrottleState new_state)
{
    if (current_state == new_state)
        return {};

     if (new_state == ThrottleState_THROTTLE_STATE_THRUST_SEQ){
        start_thrust_sequence();
    }
    current_state = new_state;

    LOG_INF("Changed State to %s", get_state_name(current_state));

    return {};
}

std::expected<void, Error> ThrottleHornetModule::init()
{

    change_state(ThrottleState_THROTTLE_STATE_IDLE);

    return {};
}

// std::optional<std::pair<AnalogSensorReadings, float>> analog_sensors_readings
ThrottleHornetStateOutput ThrottleHornetModule::step_control_loop(DataPacket& data)
{
    int64_t current_time = k_uptime_get();
    uint64_t start_cycle = k_cycle_get_64();
    ThrottleHornetStateOutput out{};

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (current_state) {
    case ThrottleState_THROTTLE_STATE_IDLE: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = idle_data;
        out = idle_out;
        break;
    }

    case ThrottleState_THROTTLE_STATE_THRUST_PRIMED: {
        auto [thrust_primed_out, thrust_primed_data] = idle_tick();
        thrust_primed_out.next_state = ThrottleState_THROTTLE_STATE_THRUST_PRIMED;
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = thrust_primed_data;
        out = thrust_primed_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_THRUST_SEQ: {
        auto [thrust_out, thrust_data] = thrust_sequence_tick(data.analog_sensors, current_time);
        data.which_throttle_state_data = DataPacket_throttle_hornet_thrust_sequence_data_tag;
        data.throttle_state_data.throttle_hornet_thrust_sequence_data = thrust_data;
        out = thrust_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_FLIGHT: {
        auto [flight_out, flight_data] = flight_tick(data.analog_sensors);
        data.which_throttle_state_data = DataPacket_throttle_flight_data_tag;
        data.throttle_state_data.throttle_flight_data = flight_data;
        out = flight_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_ABORT: {
        auto [abort_out, abort_data] = abort_tick(current_time);
        data.which_throttle_state_data = DataPacket_throttle_abort_data_tag;
        data.throttle_state_data.throttle_abort_data = abort_data;
        out = abort_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    data.which_throttle_actuator_data = DataPacket_throttle_hornet_data_tag;
    data.throttle_state_output = out;

    // TODO: how to pass this upward? it shouldnt change its own state, controller should
    change_state(out.next_state);

    data.throttle_state = current_state;
}


std::pair<ThrottleHornetStateOutput, ThrottleIdleData> ThrottleHornetModule::idle_tick()
{
    ThrottleHornetStateOutput out{};
    ThrottleIdleData data{};
    out.power_on = true;
    out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    return {out, data};
}

std::pair<ThrottleHornetStateOutput, ThrottleFlightData> ThrottleHornetModule::flight_tick(const AnalogSensorReadings& analog_sensors)
{
    ThrottleHornetStateOutput out{};
    ThrottleFlightData data{};
    //TODO: This should not be a reference
    float target_thrust = FlightController::get_z_acceleration();
    out.has_thrust = true;
    out.thrust = target_thrust;
    out.power_on = true;
    out.next_state = ThrottleState_THROTTLE_STATE_FLIGHT;

    return {out, data};
}

std::pair<ThrottleHornetStateOutput, ThrottleHornetThrustSequenceData> ThrottleHornetModule::thrust_sequence_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time)
{
    ThrottleHornetStateOutput out{};
    ThrottleHornetThrustSequenceData data{};

    float elapsed_time = current_time - sequence_start_time;
    if (elapsed_time > thrust_sequence_total_time_ms) {
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
        return {out, data};
    }

    auto target_result = throttle_thrust_trace.sample(elapsed_time);
    if (!target_result) {
        LOG_ERR("Failed to sample thrust_trace: %s", target_result.error().build_message().c_str());
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
        return {out, data};
    }

    float target_thrust_lbf = *target_result;
    data.target_thrust = target_thrust_lbf;

    out.next_state = ThrottleState_THROTTLE_STATE_THRUST_SEQ;
    return {out, data};
}

std::pair<ThrottleHornetStateOutput, ThrottleAbortData> ThrottleHornetModule::abort_tick(uint32_t current_time)
{
    ThrottleHornetStateOutput out{};
    ThrottleAbortData data{};

    out.power_on = true;

    if (current_time - abort_entry_time > 500) {
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    } else {
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
    }

    return {out, data};
}



std::expected<void, Error> ThrottleHornetModule::load_thrust_sequence(const ThrottleLoadThrustSequenceRequest& req)
{
    LOG_INF("Received load thrust sequence request");

    auto result = throttle_thrust_trace.load(req.thrust_trace_lbf);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid thrust trace"));

    thrust_sequence_total_time_ms = req.thrust_trace_lbf.total_time_ms;

    change_state(ThrottleState_THROTTLE_STATE_THRUST_PRIMED);


    return {};
}

std::expected<void, Error> ThrottleHornetModule::start_thrust_sequence(const ThrottleStartThrustSequenceRequest& req)
{
    sequence_start_time = k_uptime_get();
    low_ptc_start_time_ms = 0;
    alpha = -1.0f;
    change_state(ThrottleState_THROTTLE_STATE_THRUST_SEQ);

    return {};
}

const char* ThrottleHornetModule::get_state_name(ThrottleState state)
{
    if (state == ThrottleState_THROTTLE_STATE_IDLE)
        return "Idle";
    if (state == ThrottleState_THROTTLE_STATE_THRUST_PRIMED)
        return "Thrust Primed";
    if (state == ThrottleState_THROTTLE_STATE_THRUST_SEQ)
        return "Thrust Seq";
    if (state == ThrottleState_THROTTLE_STATE_FLIGHT)
        return "Flight";
    if (state == ThrottleState_THROTTLE_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
