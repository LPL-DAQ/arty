#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <extmem.h>
#include <zephyr/storage/flash_map.h>
#include <string.h>

/* Place test data in EXTMEM */
//__attribute__((section(".ext_rodata")))
//__attribute__((used))
//const uint8_t extmem_test_table[1024] = {1};

int extmem_enable_mmap(void)
{
    const struct device *flash =
        DEVICE_DT_GET(DT_NODELABEL(mx25l25645g));

    if (!device_is_ready(flash)) {
        printk("QSPI not ready\n");
        return -ENODEV;
    }

    printk("QSPI ready\n");

    const struct tables_header *hdr =
        (const struct tables_header *)EXTMEM_BASE;

    if (hdr->magic != TABLE_MAGIC) {
        printk("Invalid table magic: %08X\n", hdr->magic);
        return -EINVAL;
    }

    printk("Table magic OK\n");
    printk("Version: %u\n", hdr->version);
    printk("Table count: %u\n", hdr->table_count);

    const struct table_entry *dir =
        (const struct table_entry *)
        (EXTMEM_BASE + hdr->directory_offset);

    for (uint32_t i = 0; i < hdr->table_count; i++) {

        const uint8_t *table =
            (const uint8_t *)
            (EXTMEM_BASE + hdr->data_offset + dir[i].offset);

        printk("Table ID: %u\n", dir[i].id);
        printk("Size: %u\n", dir[i].size);
        printk("First byte: %02X\n", table[0]);
    }

    return 0;
}

int extmem_write_test_pattern(void)
{
    const struct device *flash =
        DEVICE_DT_GET(DT_NODELABEL(mx25l25645g));

    if (!device_is_ready(flash)) {
        printk("QSPI not ready\n");
        return -ENODEV;
    }

    printk("Erasing external flash...\n");

    /* erase first 4KB sector */
    int rc = flash_erase(flash, 0, 4096);
    if (rc) {
        printk("flash_erase failed: %d\n", rc);
        return rc;
    }

    uint8_t buf[1024];
    memset(buf, 0x5A, sizeof(buf));

    printk("Writing external flash...\n");

    rc = flash_write(flash, 0, buf, sizeof(buf));
    if (rc) {
        printk("flash_write failed: %d\n", rc);
        return rc;
    }

    printk("Write success!\n");

    return 0;
}

int extmem_write_chunk(const uint8_t *data,
                       size_t size,
                       uint32_t flash_offset)
{
    const struct device *flash =
        DEVICE_DT_GET(DT_NODELABEL(mx25l25645g));

    if (!device_is_ready(flash)) {
        return -ENODEV;
    }

    const uint32_t sector_size = 4096U;
    const uint32_t flash_size = 0x02000000U; /* 32MB */

    /* bounds check */
    if ((uint64_t)flash_offset + size > flash_size) {
        return -EINVAL;
    }

    /* compute sectors to erase */
    uint32_t start_sector = flash_offset / sector_size;
    uint32_t end_sector = (flash_offset + size - 1) / sector_size;

    for (uint32_t s = start_sector; s <= end_sector; s++) {
        uint32_t erase_addr = s * sector_size;

        int rc = flash_erase(flash, erase_addr, sector_size);
        if (rc) {
            printk("extmem: erase failed addr=0x%08X rc=%d\n", erase_addr, rc);
            return rc;
        }
    }

    /* write in smaller chunks if backend requires (flash_write may accept any len) */
    size_t written = 0;
    while (written < size) {
        size_t chunk = size - written;
        if (chunk > 4096) chunk = 4096;
        int rc = flash_write(flash, flash_offset + written, data + written, chunk);
        if (rc) {
            printk("extmem: write failed offset=%u rc=%d\n",
                   (unsigned)(flash_offset + written), rc);
            return rc;
        }
        written += chunk;
    }

    return 0;
}

/* ============================= */
/* Tables blob access functions  */
/* ============================= */

const struct tables_header *extmem_get_header(void)
{
    const struct tables_header *hdr =
        (const struct tables_header *)EXTMEM_BASE;

    if (hdr->magic != TABLE_MAGIC) {
        printk("Invalid tables magic: %08X\n", hdr->magic);
        return NULL;
    }

    return hdr;
}

const struct table_entry *extmem_get_directory(void)
{
    const struct tables_header *hdr = extmem_get_header();
    if (!hdr) {
        return NULL;
    }

    return (const struct table_entry *)
        (EXTMEM_BASE + hdr->directory_offset);
}

const uint8_t *extmem_get_table_by_index(uint32_t index)
{
    const struct tables_header *hdr = extmem_get_header();
    if (!hdr) {
        return NULL;
    }

    if (index >= hdr->table_count) {
        printk("Table index out of range\n");
        return NULL;
    }

    const struct table_entry *dir = extmem_get_directory();

    return (const uint8_t *)
        (EXTMEM_BASE + hdr->data_offset + dir[index].offset);
}

const uint8_t *extmem_get_table_by_id(uint32_t id)
{
    const struct tables_header *hdr = extmem_get_header();
    if (!hdr) {
        return NULL;
    }

    const struct table_entry *dir = extmem_get_directory();

    for (uint32_t i = 0; i < hdr->table_count; i++) {
        if (dir[i].id == id) {
            return (const uint8_t *)
                (EXTMEM_BASE + hdr->data_offset + dir[i].offset);
        }
    }

    printk("Table ID not found\n");
    return NULL;
}

int extmem_test_tables(void)
{
    const struct tables_header *hdr =
        (const struct tables_header *)EXTMEM_BASE;

    printk("Testing external tables...\n");

    if (hdr->magic != TABLE_MAGIC) {
        printk("Invalid magic: %08X\n", hdr->magic);
        return -EINVAL;
    }

    printk("Magic OK\n");
    printk("Version: %u\n", hdr->version);
    printk("Table count: %u\n", hdr->table_count);

    const struct table_entry *dir =
        (const struct table_entry *)(EXTMEM_BASE + hdr->directory_offset);

    if (hdr->table_count > 0) {
        const uint8_t *table0 =
            (const uint8_t *)(EXTMEM_BASE +
                              hdr->data_offset +
                              dir[0].offset);

        printk("Table0 size: %u\n", dir[0].size);
        printk("Table0 first byte: %02X\n", table0[0]);
    }

    return 0;
}
