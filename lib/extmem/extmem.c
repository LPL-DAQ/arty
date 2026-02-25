#include <zephyr/kernel.h>
#include "extmem.h"

int extmem_enable_mmap(void)
{
    printk("extmem: mmap not yet wired to HAL\n");
    return 0;
}
