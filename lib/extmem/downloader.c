/* lib/extmem/downloader.c */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <core_cm7.h> /* for CoreDebug */

#include "extmem.h"  /* extmem_write_chunk prototype, header has externs */

/* Host-visible RAM symbols. Place in .noinit so host-written values survive resets if needed */
volatile uint32_t downloader_buf;
volatile uint32_t downloader_size;
volatile uint32_t downloader_flash_offset;
volatile uint32_t downloader_pending;
volatile uint32_t downloader_status;

/* Return true if debugger is attached (CoreDebug DHCSR.C_DEBUGEN). */
static inline bool debugger_is_attached(void)
{
#if defined(CoreDebug) && defined(CoreDebug_DHCSR_C_DEBUGEN_Msk)
    return (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0;
#else
    return false;
#endif
}

/* Public API — called periodically in main loop or thread */
void downloader_task_check_and_run(void)
{
    /* Only allow downloader when debugger is attached */
    //if (!debugger_is_attached()) {
        //return;   /* normal mode: do nothing */
    //}

    /* Debugger attached — require explicit request from host */
    if (downloader_pending != 1) {
        return;
    }

    downloader_status = 1;  /* busy */

    uint8_t *buf = (uint8_t *)(uintptr_t)downloader_buf;
    size_t size = (size_t)downloader_size;
    uint32_t flash_off = downloader_flash_offset;

    if (buf == NULL || size == 0) {
        downloader_status = 3; /* error */
        downloader_pending = 0;
        return;
    }

    printk("downloader: writing %u bytes to flash offset 0x%08X from RAM %p\n",
           (unsigned)size, (unsigned)flash_off, (void *)buf);

    int rc = extmem_write_chunk(buf, size, flash_off);
    if (rc == 0) {
        downloader_status = 2; /* done */
        printk("downloader: write complete\n");
    } else {
        downloader_status = 3; /* error */
        printk("downloader: write failed rc=%d\n", rc);
    }

    downloader_pending = 0; /* clear request */
}
