#ifndef APP_ABORT_STATE_H
#define APP_ABORT_STATE_H

#include "State.h"

class AbortState : public State {
public:
    void init() override;
    void run(const Sensors& sensors) override;
    void end() override;
    SystemState get_state_enum() const override { return SystemState_STATE_ABORT; }

    static AbortState& get() { static AbortState instance; return instance; }
};

#endif // APP_ABORT_STATE_H
