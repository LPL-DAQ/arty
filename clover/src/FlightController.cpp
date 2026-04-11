#include "FlightController.h"
#include "sensors/AnalogSensors.h"
#include "ControllerConfig.h"
#include "config.h"
#include "flight/FlightStateAbort.h"
#include "flight/FlightStateIdle.h"
#include "flight/FlightStateTakeoff.h"
#include "flight/FlightStateFlightSeq.h"
#include "flight/FlightStateLanding.h"
// TODO: properly implement off states for this and other controllers
#include "flight/FlightStateOff.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(FlightController, LOG_LEVEL_INF);

std::expected<void, Error> FlightController::change_state(FlightState new_state)
{
    if (current_state == new_state)
        return {};

    switch (new_state) {
    case FlightState_FLIGHT_STATE_IDLE:
        current_state = new_state;
        FlightStateIdle::init();
        break;

    case FlightState_FLIGHT_STATE_TAKEOFF:
        if (current_state != FlightState_FLIGHT_STATE_IDLE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Takeoff, must be in Idle", get_state_name(current_state)));
        }
        sequence_start_time = k_uptime_get();
        current_state = new_state;
        FlightStateTakeoff::init();
        break;

    case FlightState_FLIGHT_STATE_FLIGHT_SEQ:
        if (current_state != FlightState_FLIGHT_STATE_TAKEOFF) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to FlightSeq, must be in Takeoff", get_state_name(current_state)));
        }
        sequence_start_time = k_uptime_get();
        current_state = new_state;
        FlightStateFlightSeq::init(0.0f);
        break;

    case FlightState_FLIGHT_STATE_LANDING:
        if (current_state != FlightState_FLIGHT_STATE_FLIGHT_SEQ) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Landing, must be in FlightSeq", get_state_name(current_state)));
        }
        sequence_start_time = k_uptime_get();
        current_state = new_state;
        FlightStateLanding::init();
        break;

    case FlightState_FLIGHT_STATE_ABORT:
        if (current_state != FlightState_FLIGHT_STATE_TAKEOFF && current_state != FlightState_FLIGHT_STATE_FLIGHT_SEQ && current_state != FlightState_FLIGHT_STATE_LANDING) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Abort, must be in Takeoff, FlightSeq, or Landing", get_state_name(current_state)));
        }
        abort_entry_time = k_uptime_get();
        FlightStateAbort::init();
        current_state = new_state;
        break;

    case FlightState_FLIGHT_STATE_OFF:
        if (current_state != FlightState_FLIGHT_STATE_IDLE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Off, must be in Idle", get_state_name(current_state)));
        }
        current_state = new_state;
        FlightStateOff::init();
        break;

    default:
        break;
    }

    LOG_INF("Changed Flight State to %s", get_state_name(current_state));

    return {};
}

std::expected<void, Error> FlightController::init()
{
    LOG_INF("Initializing FlightController state");
    auto ret = change_state(FlightState_FLIGHT_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change flight state to idle"));
    }
    return {};
}

void FlightController::step_control_loop(DataPacket& data, std::optional<std::pair<AnalogSensorReadings, float>> analog_sensors_readings)
{
    int64_t current_time = k_uptime_get();
    FlightStateOutput out{};

    switch (current_state) {
    case FlightState_FLIGHT_STATE_IDLE: {
        auto [idle_out, idle_data] = FlightStateIdle::tick();
        data.which_flight_state_data = DataPacket_flight_idle_data_tag;
        data.flight_state_data.flight_idle_data = idle_data;
        out = idle_out;
        break;
    }
    case FlightState_FLIGHT_STATE_TAKEOFF: {
        auto [takeoff_out, takeoff_data] = FlightStateTakeoff::tick(current_time, sequence_start_time);
        data.which_flight_state_data = DataPacket_flight_takeoff_data_tag;
        data.flight_state_data.flight_takeoff_data = takeoff_data;
        out = takeoff_out;
        break;
    }
    case FlightState_FLIGHT_STATE_FLIGHT_SEQ: {
        auto [seq_out, seq_data] = FlightStateFlightSeq::tick(data.analog_sensors, current_time, sequence_start_time);
        data.which_flight_state_data = DataPacket_flight_sequence_data_tag;
        data.flight_state_data.flight_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case FlightState_FLIGHT_STATE_LANDING: {
        auto [landing_out, landing_data] = FlightStateLanding::tick(current_time, sequence_start_time);
        data.which_flight_state_data = DataPacket_flight_landing_data_tag;
        data.flight_state_data.flight_landing_data = landing_data;
        out = landing_out;
        break;
    }
    case FlightState_FLIGHT_STATE_ABORT: {
        auto [abort_out, abort_data] = FlightStateAbort::tick(current_time, abort_entry_time);
        data.which_flight_state_data = DataPacket_flight_abort_data_tag;
        data.flight_state_data.flight_abort_data = abort_data;
        out = abort_out;
        break;
    }
    case FlightState_FLIGHT_STATE_OFF: {
        auto [off_out, off_data] = FlightStateOff::tick();
        data.which_flight_state_data = DataPacket_flight_off_data_tag;
        data.flight_state_data.flight_off_data = off_data;
        out = off_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = FlightStateIdle::tick();
        data.which_flight_state_data = DataPacket_flight_idle_data_tag;
        data.flight_state_data.flight_idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    auto ret = change_state(out.next_state);
    if (!ret.has_value()) {
        LOG_ERR("Error while changing flight state: %s", ret.error().build_message().c_str());
    }

    data.flight_state_output = out;
    data.flight_state = current_state;

    // TODO: take the values from the actual state outputs
    FlightControllerOutput flight_out = FlightControllerOutput_init_default;
    flight_out.z_acceleration = 0.0f;
    flight_out.x_angular_acceleration = 0.0f;
    flight_out.y_angular_acceleration = 0.0f;
    flight_out.roll_position = 0.0f;

    current_output = flight_out;
    data.flight_controller_output = current_output;
}

const FlightControllerOutput& FlightController::get_output()
{
    return current_output;
}

float FlightController::get_z_acceleration()
{
    return current_output.z_acceleration;
}

float FlightController::get_x_angular_acceleration()
{
    return current_output.x_angular_acceleration;
}

float FlightController::get_y_angular_acceleration()
{
    return current_output.y_angular_acceleration;
}

float FlightController::get_roll_position()
{
    return current_output.roll_position;
}

const char* FlightController::get_state_name(FlightState state)
{
    if (state == FlightState_FLIGHT_STATE_IDLE)
        return "Idle";
    if (state == FlightState_FLIGHT_STATE_ABORT)
        return "Abort";
    if (state == FlightState_FLIGHT_STATE_TAKEOFF)
        return "Takeoff";
    if (state == FlightState_FLIGHT_STATE_FLIGHT_SEQ)
        return "FlightSeq";
    if (state == FlightState_FLIGHT_STATE_LANDING)
        return "Landing";
    if (state == FlightState_FLIGHT_STATE_OFF)
        return "Off";
    return "Unknown State";
}

