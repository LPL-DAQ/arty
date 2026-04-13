#ifndef APP_TVC_HORNET_MODULE_H
#define APP_TVC_HORNET_MODULE_H

#include "../sensors/AnalogSensors.h"
#include "../Error.h"
#include "../Trace.h"
#include "clover.pb.h"
#include <expected>
#include <zephyr/kernel.h>

typedef TVCState TVCState;

class TVCHornetModule {
public:


    static inline TVCState current_state = TVCState_TVC_STATE_IDLE;
    static inline uint32_t abort_entry_time = 0;
    static inline uint32_t sequence_start_time = 0;


    static TVCState state()
    {
        return current_state;
    }

    static TVCHornetStateOutput step_control_loop(DataPacket& data);
    // Request handlers
    static std::expected<void, Error> load_sequence(const TVCLoadSequenceRequest& req);
    static std::expected<void, Error> start_sequence();

    static std::expected<void, Error> change_state(TVCState new_state);
    static const char* get_state_name(TVCState state);
    TVCHornetModule() = delete;  // Explicitly prevent instantiation

private:
    static std::pair<TVCHornetStateOutput, TVCIdleData> idle_tick();
    static std::pair<TVCHornetStateOutput, TVCIdleData> trace_primed_tick();
    static std::pair<TVCHornetStateOutput, TVCSequenceData> sequence_tick(int64_t current_time, int64_t start_time);
    static std::pair<TVCHornetStateOutput, TVCFlightData> flight_tick(const AnalogSensorReadings& analog_sensors);
    static std::pair<TVCHornetStateOutput, TVCAbortData> abort_tick(uint32_t current_time, uint32_t entry_time);

    static inline uint32_t udp_sequence_number = 0;
};

extern struct k_msgq telemetry_msgq;

#endif  // APP_TVC_HORNET_MODULE_H
