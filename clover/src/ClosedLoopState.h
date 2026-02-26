#ifndef APP_CLOSED_LOOP_STATE_H
#define APP_CLOSED_LOOP_STATE_H

#include "State.h"

class ClosedLoopState : public State {
public:
    void init() override;
    void run(const Sensors& sensors) override;
    void end() override;

    SystemState get_state_enum() const override { return SystemState_STATE_CLOSED_LOOP_THROTTLE; }

    static ClosedLoopState& get() { static ClosedLoopState instance; return instance; }
};

#endif // APP_CLOSED_LOOP_STATE_H
