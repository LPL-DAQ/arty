#ifndef APP_STATE_H
#define APP_STATE_H

#include "clover.pb.h"

// Parent Abstract Class
class State {
public:
    virtual ~State() = default;
    virtual void init() = 0;
    virtual void run(const Sensors& sensors) = 0;
    virtual void end() = 0;

    // Helps Controller.cpp easily pack the current state into telemetry
    virtual SystemState get_state_enum() const = 0;
};

#endif // APP_STATE_H
