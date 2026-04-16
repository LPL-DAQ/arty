#ifndef APP_RCS_HORNET_MODULE_H
#define APP_RCS_HORNET_MODULE_H

#include "../sensors/AnalogSensors.h"
#include "../Error.h"
#include "../Trace.h"
#include "clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>

typedef RCSState RCSState;

namespace RCSHornetModule {

    static inline RCSState current_state = RCSState_RCS_STATE_IDLE;
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;

    RCSState state();

    void step_control_loop(DataPacket& data);
    // Request handlers
    std::expected<void, Error> load_valve_sequence(const RCSLoadValveSequenceRequest& req);
    std::expected<void, Error> load_roll_sequence(const RCSLoadRollSequenceRequest& req);
    std::expected<void, Error> start_valve_sequence();
    std::expected<void, Error> start_roll_sequence();

    std::expected<void, Error> change_state(RCSState new_state);
    const char* get_state_name(RCSState state);

    std::pair<RCSHornetStateOutput, RCSIdleData> idle_tick();
    std::pair<RCSHornetStateOutput, RCSValveSequenceData> valve_sequence_tick(int64_t current_time, int64_t start_time);
    std::pair<RCSHornetStateOutput, RCSRollSequenceData> roll_sequence_tick(EstimatedState& state, int64_t current_time, int64_t start_time);
    std::pair<RCSHornetStateOutput, RCSFlightData> flight_tick(EstimatedState& state, FlightStateOutput& desired_roll_position);
    std::pair<RCSHornetStateOutput, RCSAbortData> abort_tick(uint32_t current_time, uint32_t entry_time);

    static inline uint32_t udp_sequence_number = 0;

} // namespace RCSHornetModule

extern struct k_msgq telemetry_msgq;

#endif  // APP_RCS_HORNET_MODULE_H
