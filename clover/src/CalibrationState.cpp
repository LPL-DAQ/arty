#include "CalibrationState.h"



void CalibrationState::init() {
    // Controller handles actuation now
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
            seek_hardstop(&out, fuel_pos, fuel_pos_enc, lox_pos, lox_pos_enc);
            break;
        case CalPhase::BACK_OFF:
            back_off(&out, timestamp, fuel_pos_enc, lox_pos_enc);
            break;
        case CalPhase::END_MOVEMENT:
            end_movement(&out, timestamp, fuel_pos, fuel_pos_enc, lox_pos, lox_pos_enc);
            break;
        case CalPhase::POWER_OFF:
            power_off(&out, timestamp, fuel_pos, fuel_pos_enc, lox_pos, lox_pos_enc);
            break;
        case CalPhase::REPOWER:
            repower(&out, timestamp, fuel_pos, fuel_pos_enc, lox_pos, lox_pos_enc);
            break;
        case CalPhase::COMPLETE:
             complete(&out, timestamp, fuel_pos, fuel_pos_enc, lox_pos, lox_pos_enc);
            break;
        case CalPhase::ERROR:
             error(&out, timestamp, fuel_pos, fuel_pos_enc, lox_pos, lox_pos_enc);
             break;
        default:
            break;
    }
    if (rep_counter >= num_reps) {
        phase = CalPhase::StartPowerCycle;
    }
    out.next_state = SystemState_STATE_CALIBRATION;

}
void CalibrationState::seek_hardstop(ControllerOutput* out, float fuel_pos,float fuel_pos_enc,float lox_pos, float lox_pos_enc) {
    out.set_fuel = true;
    out.set_lox = true;
    if (!fuel_found_stop && std::abs(fuel_pos - fuel_pos_enc) <= error_limit) {
            out.fuel_valve_target = fuel_pos + step_size / motion_tracker; // move towards stop, but slow down in later loops
    } else { // when it reaches
        fuel_found_stop = true;
        fuel_hardstop_position = fuel_pos_enc;
        out.fuel_valve_target = fuel_pos_enc; // hold position once we find the hardstop
    }
    // if loxside hasnt reached
    if (!lox_found_stop && std::abs(lox_pos - lox_pos_enc) <= error_limit) {
        out.lox_valve_target = lox_pos + step_size / motion_tracker;
    } else { // when it reaches
        lox_found_stop = true;
        lox_hardstop_position = lox_pos_enc;
        out.lox_valve_target = lox_pos_enc; // hold position once we find the hardstop
    }
    // if both reached, move away from stop
    if (fuel_found_stop && lox_found_stop) {
        out.fuel_valve_target =  fuel_pos_enc; // move back a bit
        out.lox_valve_target = lox_pos_enc;
        fuel_found_stop = false;
        lox_found_stop = false;
        phase = CalPhase::BackOff;
        rep_counter++;
    }

}


void CalibrationState::back_off(ControllerOutput* out, uint32_t timestamp ,float fuel_pos_enc,float lox_pos_enc) {
    // if moving away from hardstop
    out.fuel_valve_target = fuel_pos_enc - step_size / motion_tracker;
    out.lox_valve_target = lox_pos_enc - step_size / motion_tracker;
    // if both valves have moved away from hardstop enough, move towards it again
    if (fuel_hardstop_position - fuel_pos_enc  >= backup_step / motion_tracker &&
            lox_hardstop_position - lox_pos_enc >= backup_step / motion_tracker) {
        phase = CalPhase::SeekHardstop;
    }

}

void CalibrationState::end_movement(ControllerOutput* out, uint32_t timestamp) {
    FuelValve::reset_pos(fuel_hardstop_position);
    LoxValve::reset_pos(lox_hardstop_position);

    out.set_fuel = false;
    out.set_lox = false;
    if (power_cycle_timestamp == 0){
        power_cycle_timestamp = timestamp;
    }
    else if (timestamp - pwer_cycle_timestamp >= 300) {
        phase = CalPhase::POWER_OFF;
    }
}

void CalibrationState::power_off(ControllerOutput* out, uint32_t timestamp) {
    // TRIGGER MOSFETS TO TURN OFF POWER TO VALVE DRIVERS HERE
    out.set_fuel = false;
    out.set_lox = false;
    else if (timestamp - pwer_cycle_timestamp >= 3000) {
        phase = CalPhase::REPOWER;
    }
}

void CalibrationState::repower(ControllerOutput* out, uint32_t timestamp) {
    // TRIGGER MOSFETS TO TURN ON POWER TO VALVE DRIVERS HERE
    out.set_fuel = false;
    out.set_lox = false;
    else if (timestamp - pwer_cycle_timestamp >= 3300) {
        phase = CalPhase::COMPLETE;
    }
}

void CalibrationState::complete(ControllerOutput* out) {
    out.set_fuel = false;
    out.set_lox = false;
    out.next_state = SystemState_STATE_IDLE;

}
