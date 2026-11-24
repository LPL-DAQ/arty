#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int sequence_init(void);

/* Snapshot strings refer to static storage; step is one-based, or zero before start.
 * The manual mask records successful commands, not electrical feedback. */
struct sequence_snapshot {
    const char *state;
    const char *name;
    const char *comment;
    uint32_t step;
    uint32_t total_steps;
    int error;
    uint32_t manual_override_channels;
    uint32_t sequence_requested_channels;
    uint32_t manual_powered_channels;
    uint32_t powered_channels;
    uint32_t known_channels;
};

size_t sequence_count(void);
const char *sequence_name(size_t index);
/* Start queues asynchronous work; abort may return before running-step cleanup.
 * GPIO faults remain latched until firmware restart. */
int sequence_start(const char *name);
int sequence_abort(void);
/* ON and OFF persist as overrides; release resumes sequence control. */
int sequence_manual_control(uint32_t channel, bool powered, bool release);
int sequence_manual_valve(uint32_t channel, bool powered);
struct sequence_snapshot sequence_status(void);
/* Last observed GPIO readback; no I2C or mutex acquisition. */
void sequence_cached_readback(uint32_t *known, uint32_t *powered);

#ifdef __cplusplus
}
#endif
