#include "ThrottleRanger.h"
#include "ThrottleValve.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ThrottleRanger, LOG_LEVEL_INF);

namespace ThrottleRanger {
float alpha = -1.0f;
uint32_t low_ptc_start_time_ms = 0;
float target_of = 1.2f;
}

std::expected<void, Error> ThrottleRanger::tick(ThrottleStateOutput& output, DataPacket& data){

    if (output.has_fuel_reset_pos){
        LOG_INF("Resetting fuel valve position to %f", (double)output.reset_fuel_pos);
        ThrottleRanger::fuel_reset_pos(output.reset_fuel_pos);
    }
    if (output.has_lox_reset_pos){
        LOG_INF("Resetting lox valve position to %f", (double)output.reset_lox_pos);
        ThrottleRanger::lox_reset_pos(output.reset_lox_pos);
    }


    bool has_thrust = output.has_thrust;
    bool has_fuel_pos = output.has_fuel_pos;
    bool has_lox_pos = output.has_lox_pos;
    if (has_thrust && (has_fuel_pos || has_lox_pos)) {
        return std::unexpected(Error::from_cause("Thrust output cannot be set at the same time as fuel or lox position"));
    }
    if (has_fuel_pos && output.power_on){
        FuelValve::tick(output.power_on, has_fuel_pos, output.fuel_pos);

    }
    if (has_lox_pos && output.power_on) {
        LoxValve::tick(output.power_on, has_lox_pos, output.lox_pos);
    }

    // shorthand for data tag
    ThrottleRangerData& ranger_data = data.throttle_actuator_data.throttle_ranger_data;

    if (has_thrust) {
        uint32_t current_time = k_uptime_get_32();
        // TODO: Trupple check that the abort stuff works
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

        // Actuate valves based on calculations above
        LoxValve::tick(output.power_on, has_lox_pos, output.lox_pos);
        FuelValve::tick(output.power_on, has_fuel_pos, output.fuel_pos);
    } else { // if there is no thrust commanded, reset this stuff
        low_ptc_start_time_ms = 0;
        alpha = -1.0f;
    }

    // runs no matter which state:
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
