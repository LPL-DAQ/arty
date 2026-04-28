#include "util.h"
#include <zephyr/kernel.h>

float nsec_since_cycle(uint64_t start_cycle)
{
    return k_cyc_to_ns_near64(k_cycle_get_64() - start_cycle);
}
