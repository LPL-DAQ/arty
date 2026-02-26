#ifndef APP_SEQUENCE_STATE_H
#define APP_SEQUENCE_STATE_H

#include "State.h"

class SequenceState : public State {
public:
    void init() override;
    void run(const Sensors& sensors) override;
    void end() override;
    SystemState get_state_enum() const override { return SystemState_STATE_SEQUENCE; }

    static SequenceState& get() { static SequenceState instance; return instance; }
};

#endif // APP_SEQUENCE_STATE_H
