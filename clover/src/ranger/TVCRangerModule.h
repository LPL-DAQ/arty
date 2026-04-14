#ifndef APP_TVC_RANGER_MODULE_H
#define APP_TVC_RANGER_MODULE_H

#include "../sensors/AnalogSensors.h"
#include "../Error.h"
#include "../Trace.h"
#include "clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>
#include "../ControllerConfig.h"

typedef TVCState TVCState;

namespace TVCRangerModule {

    static inline TVCState current_state = TVCState_TVC_STATE_IDLE;
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;

    static TVCState state()
    {
        return current_state;
    }

    TVCRangerStateOutput step_control_loop(DataPacket& data);
    // Request handlers
    std::expected<void, Error> load_sequence(const TVCLoadSequenceRequest& req);
    std::expected<void, Error> start_sequence();

    std::expected<void, Error> change_state(TVCState new_state);
    const char* get_state_name(TVCState state);

    std::pair<TVCRangerStateOutput, TVCIdleData> idle_tick();
    std::pair<TVCRangerStateOutput, TVCIdleData> trace_primed_tick();
    std::pair<TVCRangerStateOutput, TVCSequenceData> sequence_tick(int64_t current_time, int64_t start_time);
    std::pair<TVCRangerStateOutput, TVCFlightData> flight_tick(const AnalogSensorReadings& analog_sensors);
    std::pair<TVCRangerStateOutput, TVCAbortData> abort_tick(uint32_t current_time, uint32_t entry_time);

    static inline uint32_t udp_sequence_number = 0;

} // namespace TVCRangerModule

extern struct k_msgq telemetry_msgq;

#endif  // APP_TVC_RANGER_MODULE_H
