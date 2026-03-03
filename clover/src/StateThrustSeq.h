#ifndef APP_STATE_THRUST_SEQ_H
#define APP_STATE_THRUST_SEQ_H

#include "Controller.h"

namespace StateThrustSeq {
    void init();
    std::pair<ControllerOutput, ThrustSequenceData> tick(bool has_ptc, float ptc_pressure);
}

#endif // APP_STATE_THRUST_SEQ_H
