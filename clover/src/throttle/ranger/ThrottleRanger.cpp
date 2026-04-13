#include "ThrottleRanger.h"
#include "ThrottleValve.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ThrottleRanger, LOG_LEVEL_INF);

namespace ThrottleRanger {
enum class CalPhase {
    SEEK_HARDSTOP,
    BACK_OFF,
    END_MOVEMENT,
    POWER_OFF,
    REPOWER,
    COMPLETE,
    MEASURE,
    ERROR
};

float alpha = -1.0f;
uint32_t low_ptc_start_time_ms = 0;
float target_of = 1.2f;

static Trace fuel_trace;
static Trace lox_trace;
static bool valve_sequence_has_fuel = false;
static bool valve_sequence_has_lox = false;
static float valve_sequence_fuel_total_time_ms = -1.0f;
static float valve_sequence_lox_total_time_ms = -1.0f;

static CalPhase phase = CalPhase::SEEK_HARDSTOP;
static float fuel_target_position = 0.0f;
static float fuel_hardstop_position = 0.0f;
static bool fuel_found_stop = false;
static float lox_target_position = 0.0f;
static float lox_hardstop_position = 0.0f;
static bool lox_found_stop = false;
static float step_size = 0.002f; // in degrees, how much to move per step
static int rep_counter = 0;
static float pos_error_limit = 0.2f; // positional error limit
static float fuel_starting_error = 0.0f;
static float lox_starting_error = 0.0f;
static uint32_t power_cycle_timestamp = 0;
}


std::expected<void, Error> ThrottleRanger::tick(ThrottleStateOutput& output, DataPacket& data){

    if (output.has_reset_fuel_pos){
        LOG_INF("Resetting fuel valve position to %f", (double)output.reset_fuel_pos);
        ThrottleRanger::fuel_reset_pos(output.reset_fuel_pos);
    }
    if (output.has_reset_lox_pos){
        LOG_INF("Resetting lox valve position to %f", (double)output.reset_lox_pos);
        ThrottleRanger::lox_reset_pos(output.reset_lox_pos);
    }


    if (output.has_thrust && (output.has_fuel_pos || output.has_lox_pos)) {
        return std::unexpected(Error::from_cause("Thrust output cannot be set at the same time as fuel or lox position"));
    }


    // shorthand for data tag
    ThrottleRangerData& ranger_data = data.throttle_actuator_data.throttle_ranger_data;

    if (output.has_thrust) {
        // Actuate valves based on calculations
        auto thrust_tick_result = thrust_trace_tick(output, data);
        if (!thrust_tick_result) {
            return std::unexpected(thrust_tick_result.error().context("Error in thrust tick"));
        }
    } else { // if there is no thrust commanded, reset this stuff (equal to init)
        low_ptc_start_time_ms = 0;
        alpha = -1.0f;
    }

    // ensures that there is always a position to send
    if (!output.has_fuel_pos) {
        output.has_fuel_pos = true;
        output.fuel_pos = ThrottleValve::fuel_get_pos_internal();
    }
    if (!output.has_lox_pos) {
        output.has_lox_pos = true;
        output.lox_pos = ThrottleValve::lox_get_pos_internal();
    }

    // todo: why would it ever need to know has_lox_pos?
    LoxValve::tick(output.power_on, output.has_lox_pos, output.lox_pos);
    FuelValve::tick(output.power_on, output.has_fuel_pos, output.fuel_pos);

    ranger_data.fuel_valve = {
        .target_pos_deg = output.fuel_pos,
        .driver_setpoint_pos_deg = FuelValve::get_pos_internal(),
        .encoder_pos_deg = FuelValve::get_pos_encoder(),
        .is_on = FuelValve::get_power_on(),
    };
    ranger_data.lox_valve = {
        .target_pos_deg = output.lox_pos,
        .driver_setpoint_pos_deg = LoxValve::get_pos_internal(),
        .encoder_pos_deg = LoxValve::get_pos_encoder(),
        .is_on = LoxValve::get_power_on(),
    };

    return {};
}

std::unexpected<void, Error> ThrottleRanger::thrust_trace_tick(ThrottleStateOutput& output, DataPacket& data)
{
    uint32_t current_time = k_uptime_get_32();
    // TODO: Tripple check that the abort stuff works
    // 1. Safety: abort if PTC401 is below threshold for some time
    // TODO: is battery voltage the right thing to check here? that seems wrong
    if (data.analog_sensors.battery_voltage <= PTC401_ABORT_THRESHOLD) {
        if (low_ptc_start_time_ms == 0) {
            low_ptc_start_time_ms = current_time;
        }
        else if (current_time - low_ptc_start_time_ms > PTC401_ABORT_THRESHOLD_TIME_MS) {
            // TODO: Fix float to double cast
            LOG_ERR("PTC401 < %f for >%u ms in THRUST_SEQ, aborting.", (double)PTC401_ABORT_THRESHOLD, PTC401_ABORT_THRESHOLD_TIME_MS);
                            output.power_on = false;
            output.next_state = ThrottleState_THROTTLE_STATE_ABORT;
            return {};
        }
    }
    else {
        low_ptc_start_time_ms = 0;
    }

    // 2. Read pressures
    float ptc401_val = data.analog_sensors.battery_voltage;                                    // Adjusted value
    float pto401_val = data.analog_sensors.battery_voltage + LOX_ENGINE_INLET_LINE_LOSS_PSI;   // Adjusted value
    float pt103_val = data.analog_sensors.battery_voltage;                                      // Adjusted value
    float ptf401_val = data.analog_sensors.battery_voltage + FUEL_ENGINE_INLET_LINE_LOSS_PSI;  // Adjusted value
    float pt203_val = data.analog_sensors.battery_voltage;                                      // Adjusted value
    float ptc402_val = data.analog_sensors.battery_voltage;                                    // Adjusted value
    bool pt203_valid = (data.analog_sensors.battery_voltage >= MIN_threshold && data.analog_sensors.battery_voltage <= MAX_threshold_PT2k);
    bool ptf401_valid = (data.analog_sensors.battery_voltage >= MIN_threshold && data.analog_sensors.battery_voltage <= MAX_threshold_PT2k);
    bool pt103_valid = (data.analog_sensors.battery_voltage >= MIN_threshold && data.analog_sensors.battery_voltage <= MAX_threshold_PT2k);
    bool pto401_valid = (data.analog_sensors.battery_voltage >= MIN_threshold && data.analog_sensors.battery_voltage <= MAX_threshold_PT2k);
    bool ptc401_valid = (data.analog_sensors.battery_voltage >= MIN_threshold && data.analog_sensors.battery_voltage <= MAX_threshold_PT1k);
    bool ptc402_valid = (data.analog_sensors.battery_voltage >= MIN_threshold && data.analog_sensors.battery_voltage <= MAX_threshold_PT1k);
    float p_inj_fuel;
    float p_inj_lox;
    float p_ch;
    if (ptc401_valid && ptc402_valid) {
        // Both are healthy: Take the average
        p_ch = (ptc401_val + ptc402_val) / 2.0f;
    }
    else if (ptc401_valid) {
        // Only primary is healthy
        p_ch = ptc401_val;
    }
    else if (ptc402_valid) {
        // Only backup is healthy
        p_ch = ptc402_val;
    }
    else {
        LOG_ERR("NO PTC 401 / PTC402");

        // Both failed: Trigger Abort
                    output.power_on = false;
        output.next_state = ThrottleState_THROTTLE_STATE_ABORT;
        return {};
    }
    if (pt203_valid && ptf401_valid) {
        // Both are healthy: Take the average
        p_inj_fuel = (pt203_val + ptf401_val) / 2.0f;
    }
    else if (pt203_valid) {
        // Only primary is healthy
        p_inj_fuel = pt203_val;
    }
    else if (ptf401_valid) {
        // Only backup is healthy
        p_inj_fuel = ptf401_val;
    }
    else {
        // Both failed: Trigger Abort
        LOG_ERR("NO PT 203 / ptf401");

                    output.power_on = false;
        output.next_state = ThrottleState_THROTTLE_STATE_ABORT;
        return {};
    }
    if (pt103_valid && pto401_valid) {
        // Both are healthy: Take the average
        p_inj_lox = (pt103_val + pto401_val) / 2.0f;
    }
    else if (pt103_valid) {
        // Only primary is healthy
        p_inj_lox = pt103_val;
    }
    else if (pto401_valid) {
        // Only backup is healthy
        p_inj_lox = pto401_val;
    }
    else {
        // Both failed: Trigger Abort
        LOG_ERR("NO PT 203 / 401");

                    output.power_on = false;
        output.next_state = ThrottleState_THROTTLE_STATE_ABORT;
        return {};
    }

    // 3. Calculate mass flows
    float mdot_f = calculate_fuel_mass_flow(p_inj_fuel, p_ch);
    float mdot_lox = calculate_lox_mass_flow(p_inj_lox, p_ch);

    // 4. Clamp fuel mass flow to avoid division by zero
    float mdot_f_safe = std::max(mdot_f, 0.001f);

    // 5. Calculate O/F
    float predicted_of = mdot_lox / mdot_f_safe;

    // 6. Clamp O/F for lookup
    float of_safe = std::clamp(predicted_of, MIN_SAFE_OF, MAX_SAFE_OF);

    // 7. Predict Isp using chamber pressure and O/F
    float predicted_isp = interp2D(isp_pc_axis_internal, 29, isp_of_axis_internal, 34, isp_data_internal, p_ch, of_safe);

    // 8. Predict thrust (convert to lbf-equivalent)
    float predicted_thrust = (mdot_f + mdot_lox) * predicted_isp * EFFICIENCY * LBF_CONVERSION;
    float target_thrust_lbf = output.thrust;

    // Clamp requested O/F into safe range as well
    float target_of_safe = std::clamp(target_of, MIN_SAFE_OF, MAX_SAFE_OF);

    // 9. Compute PID
    float dt = SEC_PER_CONTROL_TICK;
    float thrust_error = target_thrust_lbf - predicted_thrust;
    float change_alpha_cmd = THRUST_KP * thrust_error;
    change_alpha_cmd *= dt;
    float clamped_change_alpha_cmd = std::clamp(change_alpha_cmd, MIN_CHANGE_ALPHA, MAX_CHANGE_ALPHA);

    // 10. Integrate PID to get alpha
    if (ThrottleRanger::alpha == -1.0f) {
        // Initialize alpha to starting guess based on Mprime
        ThrottleRanger::alpha = (target_thrust_lbf - thrust_axis_internal[0]) / (thrust_axis_internal[100 - 1] - thrust_axis_internal[0]);
    }
    ThrottleRanger::alpha += clamped_change_alpha_cmd;
    ThrottleRanger::alpha = std::clamp(ThrottleRanger::alpha, MIN_ALPHA, MAX_ALPHA);

    // 11. Plug alpha into Mprime contour
    float thrust_from_alpha = ThrottleRanger::alpha * (thrust_axis_internal[100 - 1] - thrust_axis_internal[0]) + thrust_axis_internal[0];
    float fuel_valve_cmd = interp2D(thrust_axis_internal, 100, of_axis_internal, 100, fuel_valve_grid_internal, thrust_from_alpha, target_of_safe);

    float lox_valve_cmd = interp2D(thrust_axis_internal, 100, of_axis_internal, 100, lox_valve_grid_internal, thrust_from_alpha, target_of_safe);

    // 12. Clamp valve commands to safe ranges
    fuel_valve_cmd = std::clamp(fuel_valve_cmd, MIN_VALVE_POS, MAX_VALVE_POS);
    lox_valve_cmd = std::clamp(lox_valve_cmd, MIN_VALVE_POS, MAX_VALVE_POS);

    // Populate telemetry data
    ranger_data.has_predicted_thrust = true;
    ranger_data.predicted_thrust = predicted_thrust;
    ranger_data.has_predicted_of = true;
    ranger_data.predicted_of = predicted_of;
    ranger_data.has_mdot_fuel = true;
    ranger_data.mdot_fuel = mdot_f;
    ranger_data.has_mdot_lox = true;
    ranger_data.mdot_lox = mdot_lox;
    ranger_data.has_change_alpha_cmd = true;
    ranger_data.change_alpha_cmd = change_alpha_cmd;
    ranger_data.has_clamped_change_alpha_cmd = true;
    ranger_data.clamped_change_alpha_cmd = clamped_change_alpha_cmd;
    ranger_data.has_alpha = true;
    ranger_data.alpha = alpha;
    ranger_data.has_thrust_from_alpha = true;
    ranger_data.thrust_from_alpha = thrust_from_alpha;


    // 10. Populate ThrottleControllerOutput
    output.has_fuel_pos = true;
    output.fuel_pos = fuel_valve_cmd;
    output.has_lox_pos = true;
    output.lox_pos = lox_valve_cmd;
    output.next_state = ThrottleState_THROTTLE_STATE_THRUST_SEQ;
}


std::expected<void, Error> ThrottleRanger::load_valve_sequence(const ThrottleLoadValveSequenceRequest& req)
{
    bool has_fuel = req.has_fuel_trace_deg;
    bool has_lox = req.has_lox_trace_deg;
    if (!has_fuel && !has_lox) {
        return std::unexpected(Error::from_cause("No sequences provided in load request"));
    }

    valve_sequence_has_fuel = has_fuel;
    valve_sequence_has_lox = has_lox;
    valve_sequence_fuel_total_time_ms = -1.0f;
    valve_sequence_lox_total_time_ms = -1.0f;

    if (has_fuel) {
        auto result = fuel_trace.load(req.fuel_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid fuel trace"));
        valve_sequence_fuel_total_time_ms = req.fuel_trace_deg.total_time_ms;
    }

    if (has_lox) {
        auto result = lox_trace.load(req.lox_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid lox trace"));
        valve_sequence_lox_total_time_ms = req.lox_trace_deg.total_time_ms;
    }

    return {};
}


std::expected<void, Error> ThrottleRanger::reset_valve_position(Valve valve, float new_pos_deg)
{
    switch (valve) {
    case Valve_FUEL:
        ThrottleRanger::fuel_reset_pos(new_pos_deg);
        return {};
    case Valve_LOX:
        ThrottleRanger::lox_reset_pos(new_pos_deg);
        return {};
    default:
        return std::unexpected(Error::from_cause("Unknown valve identifier provided to reset command"));
    }
}

void ThrottleRanger::set_throttle_actuator_data_tag(DataPacket& data)
{
    data.which_throttle_actuator_data = DataPacket_throttle_ranger_data_tag;
}

void ThrottleRanger::init_state(ThrottleState new_state)
{
    if (new_state == ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE) {
        init_calibrate_valve(ThrottleValve::fuel_get_pos_internal(), ThrottleValve::fuel_get_pos_encoder(), ThrottleValve::lox_get_pos_internal(), ThrottleValve::lox_get_pos_encoder());
    }
}

void ThrottleRanger::init_calibrate_valve(float fuel_pos, float fuel_pos_enc, float lox_pos, float lox_pos_enc)
{
    // The calibrate state will use these current valve positions as its starting point.
    // No further global initialization is required here.
    (void)fuel_pos;
    (void)fuel_pos_enc;
    (void)lox_pos;
    (void)lox_pos_enc;
}

float ThrottleRanger::fuel_get_pos_internal(){
    return FuelValve::get_pos_internal();
};
float ThrottleRanger::fuel_get_pos_encoder(){
    return FuelValve::get_pos_encoder();
};
float ThrottleRanger::lox_get_pos_internal(){
    return LoxValve::get_pos_internal();
};
float ThrottleRanger::lox_get_pos_encoder(){
    return LoxValve::get_pos_encoder();
};
void ThrottleRanger::fuel_reset_pos(float new_pos){
    FuelValve::reset_pos(new_pos);
};
void ThrottleRanger::lox_reset_pos(float new_pos){
    LoxValve::reset_pos(new_pos);
};
bool ThrottleRanger::fuel_get_power_on(){
    return FuelValve::get_power_on();
};
bool ThrottleRanger::lox_get_power_on(){
    return  LoxValve::get_power_on();
};
