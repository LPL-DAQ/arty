#ifndef APP_STATE_THRUST_SEQ_H
#define APP_STATE_THRUST_SEQ_H

#include "Controller.h"

namespace StateThrustSeq {
    void init();
    std::pair<ControllerOutput, ThrustSequenceData> tick(const AnalogSensors& sensors, float target_thrust_lbf, float target_of);
}

#endif // APP_STATE_THRUST_SEQ_H
