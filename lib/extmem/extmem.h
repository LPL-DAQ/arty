#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int extmem_enable_mmap(void);

extern const uint8_t extmem_test_table[1024];

#ifdef __cplusplus
}
#endif
