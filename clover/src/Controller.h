#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include "pts.h"
#include "Trace.h"
#include "State.h"
#include "IdleState.h" // Needed for inline pointer initialization
#include <zephyr/kernel.h> // Needed for msgq

typedef SystemState SystemState;

class Controller {
public:
    // Define nominal safe positions (Made public for states)
    static constexpr float DEFAULT_FUEL_POS = 81.0f;
    static constexpr float DEFAULT_LOX_POS = 74.0f;

    // Shared tracking variables (Made public for states)
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;
    static inline Trace fuel_trace;
    static inline Trace lox_trace;

    static SystemState state() { return current_state->get_state_enum(); }

    // State machine control
    static inline State* current_state = &IdleState::get();
    static inline State* previous_state = &IdleState::get();
    static void change_state(State* new_state) { current_state = new_state; }

    static void init();
    static void tick(); // The 1ms dispatcher called by the timer

    // Request handlers
    static std::expected<void, Error> handle_load_motor_sequence(const LoadMotorSequenceRequest& req);
    static std::expected<void, Error> handle_start_sequence(const StartSequenceRequest& req);
    static std::expected<void, Error> handle_halt_sequence(const HaltSequenceRequest& req);
    static std::expected<void, Error> handle_reset_valve_position(const ResetValvePositionRequest& req);

    static void trigger_abort();
    Controller() = delete; // Explicitly prevent instantiation

private:
    static inline uint32_t udp_sequence_number = 0;
    static void stream_telemetry(const Sensors& sensors);
};

// Expose the Message Queue for the Server to read from
extern struct k_msgq telemetry_msgq;

#endif  // APP_CONTROLLER_H
