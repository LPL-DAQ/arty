#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

#include "sensors/AnalogSensors.h"
#include "Error.h"
#include "Trace.h"
#include "clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>

typedef SystemState SystemState;


class Controller {
public:
    // Define nominal safe positions
    static constexpr float DEFAULT_FUEL_POS = 81.0f;
    static constexpr float DEFAULT_LOX_POS = 74.0f;
    // Shared tracking variables
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;
    static inline bool fuel_powered = true;
    static inline bool lox_powered = true;

    static inline SystemState current_state = SystemState_STATE_IDLE;
    static SystemState state()
    {
        return current_state;
    }

    static std::expected<void, Error> init();
    static void controller_step_control_loop(k_work* work);  // The 1ms dispatcher called by the timer
    static void control_loop_schedule(k_timer* timer);

    static void step_control_loop(k_work*);
    // Request handlers
    static std::expected<void, Error> handle_abort(const AbortRequest& req);
    static std::expected<void, Error> change_state(SystemState new_state);
    static const char* get_state_name(SystemState state);
    Controller() = delete;  // Explicitly prevent instantiation

private:
    static inline uint32_t udp_sequence_number = 0;
};

extern struct k_msgq telemetry_msgq;

#endif  // APP_CONTROLLER_H
