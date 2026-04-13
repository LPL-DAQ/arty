#ifndef APP_RCS_CONTROLLER_H
#define APP_RCS_CONTROLLER_H

#include "../sensors/AnalogSensors.h"
#include "../Error.h"
#include "../Trace.h"
#include "clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>

typedef RCSState RCSState;

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

    static void step_control_loop(DataPacket& data);
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
    static std::pair<RCSStateOutput, RCSIdleData> idle_tick();
    static std::pair<RCSStateOutput, RCSIdleData> valve_primed_tick();
    static std::pair<RCSStateOutput, RCSValveSequenceData> valve_sequence_tick(int64_t current_time, int64_t start_time);
    static std::pair<RCSStateOutput, RCSIdleData> roll_primed_tick();
    static std::pair<RCSStateOutput, RCSRollSequenceData> roll_sequence_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time);
    static std::pair<RCSStateOutput, RCSFlightData> flight_tick();
    static std::pair<RCSStateOutput, RCSAbortData> abort_tick(uint32_t current_time, uint32_t entry_time);

    static inline uint32_t udp_sequence_number = 0;
};

extern struct k_msgq telemetry_msgq;

#endif  // APP_RCS_CONTROLLER_H
