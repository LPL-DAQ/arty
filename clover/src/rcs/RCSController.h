#ifndef APP_RCS_CONTROLLER_H
#define APP_RCS_CONTROLLER_H

#include "../sensors/AnalogSensors.h"
#include "../Error.h"
#include "../Trace.h"
#include "clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>

typedef RCSState RCSState;

struct RCSControllerOutput {

    RCSState next_state = RCSState_RCS_STATE_IDLE;
};

class RCSController {
public:


    static inline RCSState current_state = RCSState_RCS_STATE_IDLE;
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;


    static RCSState state()
    {
        return current_state;
    }

    static std::expected<void, Error> init();

    static void step_control_loop(DataPacket& data, std::optional<std::pair<AnalogSensorReadings, float>> analog_sensors_readings);
    // Request handlers
    static std::expected<void, Error> handle_abort(const AbortRequest& req);
    static std::expected<void, Error> handle_load_valve_sequence(const RCSLoadValveSequenceRequest& req);
    static std::expected<void, Error> handle_start_valve_sequence(const RCSStartValveSequenceRequest& req);
    static std::expected<void, Error> handle_load_roll_sequence(const RCSLoadRollSequenceRequest& req);
    static std::expected<void, Error> handle_start_roll_sequence(const RCSStartRollSequenceRequest& req);
    static std::expected<void, Error> handle_unprime(const RCSUnprimeRequest& req);
    static std::expected<void, Error> handle_halt(const RCSHaltRequest& req);

    static std::expected<void, Error> change_state(RCSState new_state);
    static const char* get_state_name(RCSState state);
    RCSController() = delete;  // Explicitly prevent instantiation

private:
    static inline uint32_t udp_sequence_number = 0;
};

extern struct k_msgq telemetry_msgq;

#endif  // APP_RCS_CONTROLLER_H
