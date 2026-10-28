#pragma once
#include <stddef.h>
#include <stdint.h>

struct sequence_snapshot {
    const char *state;
    const char *name;
    const char *comment;
    uint32_t step;
    uint32_t total_steps;
    int error;
};

size_t sequence_count(void);
const char *sequence_name(size_t index);
int sequence_start(const char *name);
int sequence_stop(void);
struct sequence_snapshot sequence_status(void);
