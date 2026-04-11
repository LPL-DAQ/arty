#include "FlightController.h"
#include "sensors/AnalogSensors.h"
#include "ControllerConfig.h"
#include "config.h"
#include "flight/FlightStateAbort.h"
#include "flight/FlightStateIdle.h"
#include "flight/FlightStateTakeoff.h"
#include "flight/FlightStateFlightSeq.h"
#include "flight/FlightStateLanding.h"
#include "flight/FlightStateOff.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(FlightController, LOG_LEVEL_INF);


// TODO: make throttle, tvc, rcs change states according to how this switches from takeoff -> flight -> landing
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

std::expected<void, Error> FlightController::handle_load_sequence(const FlightLoadSequenceRequest& req)
{
    LOG_INF("Received load flight sequence request");

    if (current_state != FlightState_FLIGHT_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Cannot load flight sequence from %s, must be in Idle", get_state_name(current_state)));
    }

    if (req.y_position_trace.total_time_ms != req.x_position_trace.total_time_ms ||
        req.z_position_trace.total_time_ms != req.x_position_trace.total_time_ms ||
        req.roll_angle_trace.total_time_ms != req.x_position_trace.total_time_ms) {
        return std::unexpected(Error::from_cause("Flight sequence traces must have the same total_time_ms"));
    }

    auto result_x = FlightStateFlightSeq::get_x_trace().load(req.x_position_trace);
    if (!result_x)
        return std::unexpected(result_x.error().context("%s", "Invalid X position trace"));

    auto result_y = FlightStateFlightSeq::get_y_trace().load(req.y_position_trace);
    if (!result_y)
        return std::unexpected(result_y.error().context("%s", "Invalid Y position trace"));

    auto result_z = FlightStateFlightSeq::get_z_trace().load(req.z_position_trace);
    if (!result_z)
        return std::unexpected(result_z.error().context("%s", "Invalid Z position trace"));

    auto result_roll = FlightStateFlightSeq::get_roll_trace().load(req.roll_angle_trace);
    if (!result_roll)
        return std::unexpected(result_roll.error().context("%s", "Invalid roll angle trace"));

    FlightStateFlightSeq::init(req.x_position_trace.total_time_ms);
    return {};
}

std::expected<void, Error> FlightController::handle_start_sequence(const FlightStartSequenceRequest& req)
{
    LOG_INF("Received start flight sequence request");
    sequence_start_time = k_uptime_get();
    auto ret = change_state(FlightState_FLIGHT_STATE_TAKEOFF);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to takeoff"));
    }
    return {};
}

std::expected<void, Error> FlightController::handle_halt(const FlightHaltRequest& req)
{
    LOG_INF("Received halt request");
    auto ret = change_state(FlightState_FLIGHT_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}


void FlightController::step_control_loop(DataPacket& data)
{
    int64_t current_time = k_uptime_get();

    switch (current_state) {
    case FlightState_FLIGHT_STATE_IDLE: {
        auto [idle_out, idle_data] = FlightStateIdle::tick();
        data.which_flight_state_data = DataPacket_flight_idle_data_tag;
        data.flight_state_data.flight_idle_data = idle_data;
        current_output = idle_out;
        break;
    }
    case FlightState_FLIGHT_STATE_TAKEOFF: {
        auto [takeoff_out, takeoff_data] = FlightStateTakeoff::tick(current_time, sequence_start_time);
        data.which_flight_state_data = DataPacket_flight_takeoff_data_tag;
        data.flight_state_data.flight_takeoff_data = takeoff_data;
        current_output = takeoff_out;
        break;
    }
    case FlightState_FLIGHT_STATE_FLIGHT_SEQ: {
        auto [seq_out, seq_data] = FlightStateFlightSeq::tick(data.analog_sensors, current_time, sequence_start_time);
        data.which_flight_state_data = DataPacket_flight_sequence_data_tag;
        data.flight_state_data.flight_sequence_data = seq_data;
        current_output = seq_out;
        break;
    }
    case FlightState_FLIGHT_STATE_LANDING: {
        auto [landing_out, landing_data] = FlightStateLanding::tick(current_time, sequence_start_time);
        data.which_flight_state_data = DataPacket_flight_landing_data_tag;
        data.flight_state_data.flight_landing_data = landing_data;
        current_output = landing_out;
        break;
    }
    case FlightState_FLIGHT_STATE_ABORT: {
        auto [abort_out, abort_data] = FlightStateAbort::tick(current_time, abort_entry_time);
        data.which_flight_state_data = DataPacket_flight_abort_data_tag;
        data.flight_state_data.flight_abort_data = abort_data;
        current_output = abort_out;
        break;
    }
    case FlightState_FLIGHT_STATE_OFF: {
        auto [off_out, off_data] = FlightStateOff::tick();
        data.which_flight_state_data = DataPacket_flight_off_data_tag;
        data.flight_state_data.flight_off_data = off_data;
        current_output = off_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = FlightStateIdle::tick();
        data.which_flight_state_data = DataPacket_flight_idle_data_tag;
        data.flight_state_data.flight_idle_data = idle_data;
        current_output = idle_out;
        break;
    }
    }

    auto ret = change_state(out.next_state);
    if (!ret.has_value()) {
        LOG_ERR("Error while changing flight state: %s", ret.error().build_message().c_str());
    }

    data.flight_state_output = out;
    data.flight_state = current_state;

    data.flight_controller_output = current_output;
}

const FlightStateOutput& FlightController::get_output()
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
// TODO: Flight Controller Server Commands and Handlers
