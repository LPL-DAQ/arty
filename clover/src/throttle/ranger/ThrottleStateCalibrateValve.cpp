#include "ThrottleStateCalibrateValve.h"
#include "ThrottleRanger.h"
#include <zephyr/logging/log.h>

#include <array>
LOG_MODULE_REGISTER(StateCalibrateValve);


namespace {
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



    static CalPhase phase = CalPhase::SEEK_HARDSTOP;
    float fuel_target_position = 0.0f;
    float fuel_hardstop_position = 0.0f;
    bool fuel_found_stop = false;
    float lox_target_position = 0.0f;
    float lox_hardstop_position = 0.0f;
    bool lox_found_stop = false;
    float step_size = 0.002f; // in degrees, how much to move per step
    int rep_counter = 0;
    float pos_error_limit = 0.2f; // positional error limit
    float fuel_starting_error = 0.0f;
    float lox_starting_error = 0.0f;

    uint32_t power_cycle_timestamp = 0;
}



void StateCalibrateValve::init(float fuel_pos, float fuel_pos_enc, float lox_pos, float lox_pos_enc) {
    // Controller handles actuation now
    phase = CalPhase::SEEK_HARDSTOP;
    rep_counter = 0;
    fuel_found_stop = false;
    lox_found_stop = false;
    fuel_hardstop_position = 0.0f;
    lox_hardstop_position = 0.0f;
    power_cycle_timestamp = 0;

    fuel_target_position = fuel_pos_enc;
    lox_target_position = lox_pos_enc;

    fuel_starting_error = fuel_pos - fuel_pos_enc;
    lox_starting_error = lox_pos - lox_pos_enc;
}

std::pair<ThrottleStateOutput, ThrottleValveCalibrationData> StateCalibrateValve::tick(uint32_t timestamp,float fuel_pos, float lox_pos,float fuel_pos_enc, float lox_pos_enc) {
    ThrottleStateOutput out{};
    ThrottleValveCalibrationData data{};

    switch (phase) {
        case CalPhase::SEEK_HARDSTOP:
            seek_hardstop(out, fuel_pos, fuel_pos_enc,
                            lox_pos, lox_pos_enc);
            break;
        case CalPhase::BACK_OFF:
            break;
        case CalPhase::END_MOVEMENT:
            end_movement(out, timestamp);
            break;
        case CalPhase::POWER_OFF:
            power_off(out, timestamp);
            break;
        case CalPhase::REPOWER:
            repower(out, timestamp);
            break;
        case CalPhase::COMPLETE:
            complete(out, timestamp);
            break;
        case CalPhase::MEASURE:
            measure(out, fuel_pos, fuel_pos_enc, lox_pos, lox_pos_enc);
            break;
        case CalPhase::ERROR:
            error(out, timestamp);
            break;
        default:
            break;
    }
    data.fuel_found_hardstop = fuel_found_stop;
    data.fuel_hardstop_pos = fuel_hardstop_position;
    data.lox_found_hardstop = lox_found_stop;
    data.lox_hardstop_pos = lox_hardstop_position;
    data.fuel_err = fuel_pos - (fuel_pos_enc+ fuel_starting_error);
    data.lox_err = lox_pos - (lox_pos_enc + lox_starting_error);
    data.cal_phase = get_phase_id();


    return std::make_pair(out, data);
}
void StateCalibrateValve::seek_hardstop(ThrottleStateOutput& out, float fuel_pos,float fuel_pos_enc,float lox_pos, float lox_pos_enc) {
    out.power_on = true;
    out.power_on = true;

    if (!lox_found_stop
        && std::abs(lox_pos - (lox_starting_error + lox_pos_enc)) <= pos_error_limit
    ) {
            lox_target_position += step_size / (rep_counter+1);
            out.has_lox_pos = true;
            out.lox_pos = lox_target_position; // move towards stop, but slow down in later loops
    } else { // when it reaches
        lox_found_stop = true;
        lox_hardstop_position = lox_pos_enc;
        out.has_lox_pos = true;
        out.lox_pos = lox_pos_enc; // hold position once we find the hardstop
    }


    if (!fuel_found_stop
        && std::abs(fuel_pos - (fuel_starting_error + fuel_pos_enc)) <= pos_error_limit
    ) {
            fuel_target_position += step_size / (rep_counter+1);
            out.has_fuel_pos = true;
            out.fuel_pos = fuel_target_position; // move towards stop, but slow down in later loops
    } else { // when it reaches
        fuel_found_stop = true;
        fuel_hardstop_position = fuel_pos_enc;
        out.has_fuel_pos = true;
        out.fuel_pos = fuel_pos_enc; // hold position once we find the hardstop
    }

    // if both reached, move away from stop
    if (fuel_found_stop && lox_found_stop) {
        fuel_found_stop = false;
        lox_found_stop = false;
        rep_counter++;
        phase = CalPhase::END_MOVEMENT;
    }
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;
}


void StateCalibrateValve::end_movement(ThrottleStateOutput& out, uint32_t timestamp) {
    ThrottleRanger::fuel_reset_pos(fuel_hardstop_position);
    ThrottleRanger::lox_reset_pos(lox_hardstop_position);

    out.power_on = true;
    out.power_on = false;
    if (power_cycle_timestamp == 0){
        power_cycle_timestamp = timestamp;
    }
    else if (timestamp - power_cycle_timestamp >= 1000) {
        phase = CalPhase::POWER_OFF;
    }
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;

}

void StateCalibrateValve::power_off(ThrottleStateOutput& out, uint32_t timestamp) {
    out.power_on = true;
    out.power_on = false;
    if (timestamp - power_cycle_timestamp >= 4000) {
        phase = CalPhase::REPOWER;
    }
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;
}

void StateCalibrateValve::repower(ThrottleStateOutput& out, uint32_t timestamp) {
    out.power_on = true;
    out.power_on = true;
    if (timestamp - power_cycle_timestamp >= 5000) {
        phase = CalPhase::COMPLETE;
    }
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;
}

void StateCalibrateValve::complete(ThrottleStateOutput& out, uint32_t timestamp) {
    out.power_on = true;
    out.power_on = true;
    out.has_fuel_pos = true;
    out.fuel_pos = 95.0f;
    out.has_lox_pos = true;
    out.lox_pos = 95.0f;
    ThrottleRanger::fuel_reset_pos(95.0f);
    ThrottleRanger::lox_reset_pos(95.0f);

    fuel_found_stop = false;
    lox_found_stop = false;
    fuel_starting_error = 0;
    lox_starting_error = 0;
    fuel_target_position = 95;
    lox_target_position = 95;

    // should be idle, but this is for testing
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;
    if (timestamp - power_cycle_timestamp >= 6500) {
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
        phase = CalPhase::COMPLETE;
    }

}

void StateCalibrateValve::error(ThrottleStateOutput& out, uint32_t timestamp) {
    // In error, turn off drivers and do not try to move
    out.power_on = true;
    out.power_on = false;

    if (power_cycle_timestamp == 0){
        power_cycle_timestamp = timestamp;
    }
}

void StateCalibrateValve::measure(ThrottleStateOutput& out, float fuel_pos,float fuel_pos_enc,float lox_pos, float lox_pos_enc) {
    out.power_on = true;
    out.power_on = true;

    if (lox_pos_enc > 10){
        lox_target_position -= step_size*3;
        out.has_lox_pos = true;
        out.lox_pos = lox_target_position; // move towards stop, but slow down in later loops
    }
    else if (!lox_found_stop
        && std::abs(lox_pos - (lox_starting_error + lox_pos_enc)) <= pos_error_limit
    ) {
        lox_target_position -= step_size;
        out.has_lox_pos = true;
        out.lox_pos = lox_target_position; // move towards stop, but slow down in later loops
    } else { // when it reaches
        lox_found_stop = true;
        lox_hardstop_position = lox_pos_enc;
        out.has_lox_pos = true;
        out.lox_pos = lox_pos_enc; // hold position once we find the hardstop
    }

    fuel_found_stop = true;
    fuel_hardstop_position = fuel_pos_enc;
    out.has_fuel_pos = true;
    out.fuel_pos = fuel_pos_enc; // hold position once we find the hardstop

    // if both reached, move away from stop
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;

    if (fuel_found_stop && lox_found_stop) {
        fuel_target_position = fuel_hardstop_position;
        lox_target_position = lox_hardstop_position;
        // LOG_INF("err: %f, pos %f, enc %f",  std::abs(lox_pos - (lox_starting_error + lox_pos_enc)), lox_pos, lox_pos_enc);
        // LOG_INF("Fuel hardstop at %f, Lox hardstop at %f", fuel_hardstop_position, lox_hardstop_position);
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    }
}

int StateCalibrateValve::get_phase_id() {

    switch (phase) {
        case CalPhase::SEEK_HARDSTOP:
            return 0;
        case CalPhase::BACK_OFF:
            return 1;
        case CalPhase::END_MOVEMENT:
            return 2;
        case CalPhase::POWER_OFF:
            return 3;
        case CalPhase::REPOWER:
            return 4;
        case CalPhase::COMPLETE:
            return 5;
        case CalPhase::MEASURE:
            return 6;
        case CalPhase::ERROR:
            return 7;
        default:
            return -1; // Unknown phase
    }
}
