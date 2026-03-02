#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include "pts.h"
#include "Trace.h"
#include <zephyr/kernel.h> // Needed for msgq

typedef SystemState SystemState;

// The pure data contract returned by every logic module
struct ControllerOutput {
    bool set_fuel = false;
    float fuel_pos = 0.0f;
    bool set_lox = false;
    float lox_pos = 0.0f;
    SystemState next_state = SystemState_STATE_IDLE;
};

class Controller {
public:
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
    static std::expected<void, Error> handle_start_closed_loop(const StartThrottleClosedLoopRequest& req);
    static void trigger_abort();
    static void change_state(SystemState new_state);

    Controller() = delete; // Explicitly prevent instantiation

private:
    static inline uint32_t udp_sequence_number = 0;
    static void stream_telemetry(const Sensors& sensors);
};

// Expose the Message Queue for the Server to read from
extern struct k_msgq telemetry_msgq;

#endif  // APP_CONTROLLER_H
