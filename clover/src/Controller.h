#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

#include "Error.h"
#include "clover.pb.h"
#include <expected>

class Controller {
public:
    enum class State {
        IDLE,           // Waiting for commands
        SEQUENCE,       // Running open-loop sequencer
        CLOSED_LOOP,    // Future: QVC control
        ABORT           // Emergency shutdown
    };

    // Singleton access to ensure only one controller manages the valves
    static Controller& get() { static Controller instance; return instance; }

    void init();
    void tick(); // The 1ms dispatcher called by the timer

    // Request handlers (called from server.cpp)
    std::expected<void, Error> handle_load_motor_sequence(const LoadMotorSequenceRequest& req);
    std::expected<void, Error> handle_start_sequence(const StartSequenceRequest& req);
    std::expected<void, Error> handle_halt_sequence(const HaltSequenceRequest& req);

    void trigger_abort();
    State current_state() const { return _state; }

private:
    Controller() = default;
    State _state = State::IDLE;
    uint32_t abort_entry_time = 0;

    // State logic handlers
    void run_idle();
    void run_sequence();
    void run_abort();
    void stream_telemetry();
};

#endif
