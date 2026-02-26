#ifndef APP_IDLE_STATE_H
#define APP_IDLE_STATE_H

#include "State.h"

class IdleState : public State {
public:
    void init() override;
    void run(const Sensors& sensors) override;
    void end() override;
    SystemState get_state_enum() const override { return SystemState_STATE_IDLE; }

    static IdleState& get() { static IdleState instance; return instance; }
};

#endif // APP_IDLE_STATE_H
