#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include "pts.h"
#include "Trace.h"
#include <zephyr/kernel.h> // Needed for msgq

// ADDED: Use the Protobuf-generated enum directly to avoid naming collisions
typedef SystemState SystemState;

class Controller {
private:
    // Define nominal safe positions
    static constexpr float DEFAULT_FUEL_POS = 81.0f;
    static constexpr float DEFAULT_LOX_POS = 74.0f;
public:
    // Removed constructor/singleton. All methods are now static.
    static SystemState state() { return _state; }

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
    // ADDED: Initialize using the proper Protobuf enum prefix
    static inline SystemState _state = SystemState_STATE_IDLE;
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;
    static inline uint32_t udp_sequence_number = 0;

    // Trace objects replace the sequencer
    static inline Trace fuel_trace;
    static inline Trace lox_trace;

    // State logic handlers
    static void run_idle(const Sensors& sensors);
    static void run_sequence(const Sensors& sensors);
    static void run_abort(const Sensors& sensors);
    static void stream_telemetry(const Sensors& sensors);
};

// Expose the Message Queue for the Server to read from
extern struct k_msgq telemetry_msgq;

#endif  // APP_CONTROLLER_H
