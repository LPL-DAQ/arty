#include "util.h"
#include <zephyr/kernel.h>

float nsec_since_cycle(uint64_t start_cycle)
{
    return (k_cycle_get_64() - start_cycle) * (1e9f / SystemCoreClock);
}
