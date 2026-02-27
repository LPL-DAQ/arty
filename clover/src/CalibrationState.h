#ifndef APP_ABORT_STATE_H
#define APP_ABORT_STATE_H

#include "Controller.h"

namespace CalibrationState {

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
    }

    static CalPhase phase = CalPhase::SeekHardstop;
    float fuel_hardstop_position = 0.0f;
    bool fuel_found_stop = false;
    float lox_hardstop_position = 0.0f;
    bool lox_found_stop = false;
    float step_size = 0.5f; // in degrees, how much to move per step
    int num_reps = 2; // number of times to hit the hard
    int rep_counter = 0;
    float error_limit = 0.5f;
    float backup_step = 0.5f;
    uint32_t power_cycle_timestamp = 0.0f;

    void init();

    ControllerOutput tick(uint32_t timestamp,float fuel_pos,float fuel_pos_enc,float lox_pos, float lox_pos_enc);
}

#endif // APP_CALIBRATION_STATE_H
