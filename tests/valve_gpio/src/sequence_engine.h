#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Snapshot strings refer to static storage; step is one-based, or zero before start.
 * The manual mask records successful commands, not electrical feedback. */
struct sequence_snapshot {
    const char *state;
    const char *name;
    const char *comment;
    uint32_t step;
    uint32_t total_steps;
    int error;
    uint32_t manual_powered_channels;
};

size_t sequence_count(void);
const char *sequence_name(size_t index);
/* Start queues asynchronous work; abort may return before running-step cleanup.
 * GPIO faults remain latched until firmware restart. */
int sequence_start(const char *name);
int sequence_abort(void);
int sequence_manual_valve(uint32_t channel, bool powered);
struct sequence_snapshot sequence_status(void);
