#ifndef APP_TVC_CONTROLLER_H
#define APP_TVC_CONTROLLER_H

#include "../sensors/AnalogSensors.h"
#include "../Error.h"
#include "../Trace.h"
#include "clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>

typedef TVCState TVCState;

struct TVCControllerOutput {

    TVCState next_state = TVCState_TVC_STATE_IDLE;
};

class TVCController {
public:


    static inline TVCState current_state = TVCState_TVC_STATE_IDLE;
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;


    static TVCState state()
    {
        return current_state;
    }

    static std::expected<void, Error> init();

    static void step_control_loop(std::optional<std::pair<AnalogSensorReadings, float>> analog_sensors_readings);
    // Request handlers
    static std::expected<void, Error> handle_abort(const AbortRequest& req);
    static std::expected<void, Error> handle_load_sequence(const TVCLoadSequenceRequest& req);
    static std::expected<void, Error> handle_start_sequence(const TVCStartSequenceRequest& req);
    static std::expected<void, Error> handle_unprime(const TVCUnprimeRequest& req);
    static std::expected<void, Error> handle_halt(const TVCHaltRequest& req);

    static std::expected<void, Error> change_state(TVCState new_state);
    static const char* get_state_name(TVCState state);
    TVCController() = delete;  // Explicitly prevent instantiation

private:
    static inline uint32_t udp_sequence_number = 0;
};

extern struct k_msgq telemetry_msgq;

#endif  // APP_TVC_CONTROLLER_H
