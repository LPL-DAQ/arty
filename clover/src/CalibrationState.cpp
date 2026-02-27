#include "CalibrationState.h"


namespace {
    enum class CalPhase {
        SEEK_HARDSTOP,
        BACK_OFF,
        END_MOVEMENT,
        POWER_OFF,
        REPOWER,
        COMPLETE,
        ERROR
    };
    static CalPhase phase = CalPhase::SEEK_HARDSTOP;
    float fuel_hardstop_position = 0.0f;
    bool fuel_found_stop = false;
    float lox_hardstop_position = 0.0f;
    bool lox_found_stop = false;
    float step_size = 0.5f; // in degrees, how much to move per step
    int num_reps = 2; // number of times to hit the hard
    int rep_counter = 0;
    float error_limit = 0.5f;
    float backup_dist = step_size * 4.0f; // backup distance is twice the step size
    uint32_t power_cycle_timestamp = 0;
}



void CalibrationState::init() {
    // Controller handles actuation now
    phase = CalPhase::SEEK_HARDSTOP;
    rep_counter = 0;
    fuel_found_stop = false;
    lox_found_stop = false;
    fuel_hardstop_position = 0.0f;
    lox_hardstop_position = 0.0f;
    power_cycle_timestamp = 0;
}



ControllerOutput CalibrationState::tick(uint32_t timestamp, float fuel_pos,float fuel_pos_enc,float lox_pos, float lox_pos_enc) {
    ControllerOutput out{};

    switch (phase) {
        case CalPhase::SEEK_HARDSTOP:
            seek_hardstop(out, fuel_pos, fuel_pos_enc, lox_pos, lox_pos_enc);
            break;
        case CalPhase::BACK_OFF:
            back_off(out, fuel_pos_enc, lox_pos_enc);
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
            complete(out);
            return out;
        case CalPhase::ERROR:
            error(out, timestamp);
            break;
        default:
            break;
    }
    out.next_state = SystemState_STATE_CALIBRATION;
    return out;
}
void CalibrationState::seek_hardstop(ControllerOutput& out, float fuel_pos,float fuel_pos_enc,float lox_pos, float lox_pos_enc) {
    out.set_fuel = true;
    out.set_lox = true;

    /***
     detection will likely need fiddling
     ideas:
     - encoder velocity
     - a counter so that it needs to be in this error for several ticks

    */

    if (!fuel_found_stop && std::abs(fuel_pos - fuel_pos_enc) <= error_limit) {
            out.fuel_pos = fuel_pos + step_size / (rep_counter+1); // move towards stop, but slow down in later loops
    } else { // when it reaches
        fuel_found_stop = true;
        fuel_hardstop_position = fuel_pos_enc;
        out.fuel_pos = fuel_pos_enc; // hold position once we find the hardstop
    }
    // if loxside hasnt reached
    if (!lox_found_stop && std::abs(lox_pos - lox_pos_enc) <= error_limit) {
        out.lox_pos = lox_pos + step_size / (rep_counter+1); // move towards stop, but slow down in later loops
    } else { // when it reaches
        lox_found_stop = true;
        lox_hardstop_position = lox_pos_enc;
        out.lox_pos = lox_pos_enc; // hold position once we find the hardstop
    }
    // if both reached, move away from stop
    if (fuel_found_stop && lox_found_stop) {
        out.fuel_pos =  fuel_pos_enc; // move back a bit
        out.lox_pos = lox_pos_enc;
        fuel_found_stop = false;
        lox_found_stop = false;
        rep_counter++;
        if (rep_counter >= num_reps) {
            phase = CalPhase::END_MOVEMENT;
        } else {
            phase = CalPhase::BACK_OFF;
        }
    }

}


void CalibrationState::back_off(ControllerOutput& out ,float fuel_pos_enc,float lox_pos_enc) {
    // if moving away from hardstop
    out.set_fuel = true;
    out.set_lox = true;
    out.fuel_pos = fuel_pos_enc - step_size / (rep_counter+1);
    out.lox_pos = lox_pos_enc - step_size / (rep_counter+1);
    // if both valves have moved away from hardstop enough, move towards it again
    if (fuel_hardstop_position - fuel_pos_enc  >= backup_dist / (rep_counter+1) &&
            lox_hardstop_position - lox_pos_enc >= backup_dist / (rep_counter+1)) {
        phase = CalPhase::SEEK_HARDSTOP;
    }

}

void CalibrationState::end_movement(ControllerOutput& out, uint32_t timestamp) {
    FuelValve::reset_pos(fuel_hardstop_position);
    LoxValve::reset_pos(lox_hardstop_position);

    out.set_fuel = false;
    out.set_lox = false;
    if (power_cycle_timestamp == 0){
        power_cycle_timestamp = timestamp;
    }
    else if (timestamp - power_cycle_timestamp >= 1000) {
        phase = CalPhase::POWER_OFF;
    }
}

void CalibrationState::power_off(ControllerOutput& out, uint32_t timestamp) {
    if (FuelValve::get_power_on()) {
        FuelValve::power_on(false);
    }
    if (LoxValve::get_power_on()) {
        LoxValve::power_on(false);
    }
    out.set_fuel = false;
    out.set_lox = false;
    if (timestamp - power_cycle_timestamp >= 4000) {
        phase = CalPhase::REPOWER;
    }
}

void CalibrationState::repower(ControllerOutput& out, uint32_t timestamp) {
    if (!FuelValve::get_power_on()) {
        FuelValve::power_on(true);
    }
    if (!LoxValve::get_power_on()) {
        LoxValve::power_on(true);
    }
    out.set_fuel = false;
    out.set_lox = false;
    if (timestamp - power_cycle_timestamp >= 5000) {
        phase = CalPhase::COMPLETE;
    }
}

void CalibrationState::complete(ControllerOutput& out) {
    out.set_fuel = false;
    out.set_lox = false;
    out.next_state = SystemState_STATE_IDLE;

}

void CalibrationState::error(ControllerOutput& out, uint32_t timestamp) {
    // In the event of an error, we want to move the valves to a safe position (fully closed)
    out.set_fuel = false;
    out.set_lox = false;

    if (power_cycle_timestamp == 0){
        power_cycle_timestamp = timestamp;
    }

    // MOSFET CODE TO TURN OFF POWER TO VALVE DRIVERS HERE

}
