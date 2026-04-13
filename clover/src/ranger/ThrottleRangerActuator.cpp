#include "ThrottleRangerActuator.h"
#include "ThrottleValve.h"
#include "../../ControllerConfig.h"
#include "../../LookupTable.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ThrottleRangerActuator, LOG_LEVEL_INF);


std::expected<void, Error> ThrottleRangerActuator::tick(ThrottleRangerStateOutput& output, ThrottleRangerActuatorData data){

    if (output.has_reset_fuel_pos){
        LOG_INF("Resetting fuel valve position to %f", (double)output.reset_fuel_pos);
        fuel_reset_pos(output.reset_fuel_pos);
    }
    if (output.has_reset_lox_pos){
        LOG_INF("Resetting lox valve position to %f", (double)output.reset_lox_pos);
        lox_reset_pos(output.reset_lox_pos);
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

    data.fuel_valve = {
        .target_pos_deg = output.fuel_pos,
        .driver_setpoint_pos_deg = FuelValve::get_pos_internal(),
        .encoder_pos_deg = FuelValve::get_pos_encoder(),
        .is_on = FuelValve::get_power_on(),
    };
    data.lox_valve = {
        .target_pos_deg = output.lox_pos,
        .driver_setpoint_pos_deg = LoxValve::get_pos_internal(),
        .encoder_pos_deg = LoxValve::get_pos_encoder(),
        .is_on = LoxValve::get_power_on(),
    };

    return {};
}

std::expected<void, Error> ThrottleRangerActuator::reset_valve_position(Valve valve, float new_pos_deg)
{
    switch (valve) {
    case Valve_FUEL:
        fuel_reset_pos(new_pos_deg);
        return {};
    case Valve_LOX:
        lox_reset_pos(new_pos_deg);
        return {};
    default:
        return std::unexpected(Error::from_cause("Unknown valve identifier provided to reset command"));
    }
}

float ThrottleRangerActuator::fuel_get_pos_internal(){
    return FuelValve::get_pos_internal();
};
float ThrottleRangerActuator::fuel_get_pos_encoder(){
    return FuelValve::get_pos_encoder();
};
float ThrottleRangerActuator::lox_get_pos_internal(){
    return LoxValve::get_pos_internal();
};
float ThrottleRangerActuator::lox_get_pos_encoder(){
    return LoxValve::get_pos_encoder();
};
void ThrottleRangerActuator::fuel_reset_pos(float new_pos){
    FuelValve::reset_pos(new_pos);
};
void ThrottleRangerActuator::lox_reset_pos(float new_pos){
    LoxValve::reset_pos(new_pos);
};
bool ThrottleRangerActuator::fuel_get_power_on(){
    return FuelValve::get_power_on();
};
bool ThrottleRangerActuator::lox_get_power_on(){
    return  LoxValve::get_power_on();
};
