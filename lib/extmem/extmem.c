/*
 * extmem.c
 *
 * Flash-backed tables access (flash_read-based, no direct EXTMEM_BASE deref)
 *
 * This version reads header/directory/table into RAM buffers using Zephyr's
 * flash API. It avoids dereferencing 0x90000000 unless you explicitly enable
 * memory-mapped mode (not implemented here).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <extmem.h>
#include <zephyr/storage/flash_map.h>
#include <string.h>
#include <stddef.h>

/* Tunable limits */
#define MAX_TABLES        128U        /* max number of directory entries we can hold */
#define MAX_TABLE_SIZE    (64 * 1024) /* 64 KB max table size for the static buffer */

/* Flash device node label (matches your DT) */
#define FLASH_NODE_LABEL mx25l25645g

/* Internal RAM caches */
static struct tables_header s_hdr;
static bool s_hdr_valid = false;

static struct table_entry s_dir[MAX_TABLES];
static bool s_dir_valid = false;

/* Buffer used to hold a single table when callers request pointer access.
 * Note: callers that use extmem_get_table_by_index() will get a pointer to
 * this buffer; contents may be overwritten by subsequent calls. If you need
 * concurrent access, change to dynamic allocation.
 */
static uint8_t s_table_buf[MAX_TABLE_SIZE];

/* Helper: get flash device */
static const struct device *get_flash_dev(void)
{
    const struct device *flash = DEVICE_DT_GET(DT_NODELABEL(FLASH_NODE_LABEL));
    return flash;
}

/* Helper: read from flash */
static int flash_read_helper(off_t offset, void *buf, size_t len)
{
    const struct device *flash = get_flash_dev();
    if (!device_is_ready(flash)) {
        printk("extmem: flash device not ready\n");
        return -ENODEV;
    }

    int rc = flash_read(flash, offset, buf, len);
    if (rc) {
        printk("extmem: flash_read failed off=0x%08X len=%u rc=%d\n",
               (unsigned)offset, (unsigned)len, rc);
    }
    return rc;
}

/* Public functions */

int extmem_init(void)
{
    /* This implementation does NOT enable memory-mapped hardware mode.
     * It simply verifies that we can read the header via flash_read().
     *
     * If you want true memory-mapped/XIP behavior, implement the platform-
     * specific QSPI memory-mapped initialization (STM32 HAL HAL_QSPI_MemoryMapped
     * or equivalent) and return 0 when successful.
     */
    int rc;
    struct tables_header hdr_tmp;

    rc = flash_read_helper(0, &hdr_tmp, sizeof(hdr_tmp));
    if (rc) {
        printk("extmem_enable_mmap: cannot read header (flash_read failed)\n");
        return rc;
    }

    if (hdr_tmp.magic != TABLE_MAGIC) {
        printk("extmem_enable_mmap: invalid magic: %08X\n", hdr_tmp.magic);
        return -EINVAL;
    }

    /* cache header so other APIs can use it */
    memcpy(&s_hdr, &hdr_tmp, sizeof(s_hdr));
    s_hdr_valid = true;
    s_dir_valid = false; /* directory not yet read */

    printk("extmem_enable_mmap: header ok (version=%u table_count=%u)\n",
           s_hdr.version, s_hdr.table_count);

    /* NOTE: does not actually put QSPI into memory-mapped mode */
    return 0;
}

int extmem_write_test_pattern(void)
{
    const struct device *flash = get_flash_dev();
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
    const struct device *flash = get_flash_dev();
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
    int rc;

    if (s_hdr_valid) {
        return &s_hdr;
    }

    /* read header from flash offset 0 */
    rc = flash_read_helper(0, &s_hdr, sizeof(s_hdr));
    if (rc) {
        s_hdr_valid = false;
        return NULL;
    }

    if (s_hdr.magic != TABLE_MAGIC) {
        printk("Invalid tables magic: %08X\n", s_hdr.magic);
        s_hdr_valid = false;
        return NULL;
    }

    /* simple sanity checks */
    if (s_hdr.table_count > MAX_TABLES) {
        printk("extmem: table_count %u > MAX_TABLES %u\n", s_hdr.table_count, MAX_TABLES);
        s_hdr_valid = false;
        return NULL;
    }

    s_hdr_valid = true;
    s_dir_valid = false;
    return &s_hdr;
}

const struct table_entry *extmem_get_directory(void)
{
    int rc;
    const struct tables_header *hdr = extmem_get_header();
    if (!hdr) {
        return NULL;
    }

    if (s_dir_valid) {
        return s_dir;
    }

    if (hdr->directory_size == 0 || hdr->table_count == 0) {
        printk("extmem: empty directory\n");
        return NULL;
    }

    size_t expect_bytes = hdr->table_count * sizeof(struct table_entry);
    if (hdr->directory_size < expect_bytes) {
        printk("extmem: directory_size (%u) smaller than expected (%u)\n",
               hdr->directory_size, (unsigned)expect_bytes);
        return NULL;
    }

    /* read directory from flash at hdr->directory_offset */
    rc = flash_read_helper(hdr->directory_offset, s_dir, expect_bytes);
    if (rc) {
        s_dir_valid = false;
        return NULL;
    }

    s_dir_valid = true;
    return s_dir;
}

/* Return pointer to a RAM buffer holding the table data.
 * The buffer is static and may be overwritten by subsequent calls.
 * Returns NULL on error.
 */
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
    if (!dir) {
        return NULL;
    }

    uint32_t tsize = dir[index].size;
    if (tsize == 0) {
        printk("extmem: table %u has zero size\n", index);
        return NULL;
    }

    if (tsize > MAX_TABLE_SIZE) {
        printk("extmem: table %u size %u > MAX_TABLE_SIZE %u\n",
               index, (unsigned)tsize, (unsigned)MAX_TABLE_SIZE);
        return NULL;
    }

    uint32_t table_flash_off = hdr->data_offset + dir[index].offset;
    int rc = flash_read_helper(table_flash_off, s_table_buf, tsize);
    if (rc) {
        return NULL;
    }

    return s_table_buf;
}

const uint8_t *extmem_get_table_by_id(uint32_t id)
{
    const struct tables_header *hdr = extmem_get_header();
    if (!hdr) {
        return NULL;
    }

    const struct table_entry *dir = extmem_get_directory();
    if (!dir) {
        return NULL;
    }

    for (uint32_t i = 0; i < hdr->table_count; i++) {
        if (dir[i].id == id) {
            uint32_t tsize = dir[i].size;
            if (tsize == 0) {
                printk("extmem: table id %u has zero size\n", id);
                return NULL;
            }
            if (tsize > MAX_TABLE_SIZE) {
                printk("extmem: table id %u size %u > MAX_TABLE_SIZE %u\n",
                       id, (unsigned)tsize, (unsigned)MAX_TABLE_SIZE);
                return NULL;
            }
            uint32_t table_flash_off = hdr->data_offset + dir[i].offset;
            int rc = flash_read_helper(table_flash_off, s_table_buf, tsize);
            if (rc) {
                return NULL;
            }
            return s_table_buf;
        }
    }

    printk("Table ID not found\n");
    return NULL;
}

int extmem_test_tables(void)
{
    const struct tables_header *hdr = extmem_get_header();
    if (!hdr) {
        printk("Testing external tables: header not found\n");
        return -EINVAL;
    }

    printk("Testing external tables (flash_read mode)...\n");
    printk("Magic OK\n");
    printk("Version: %u\n", hdr->version);
    printk("Table count: %u\n", hdr->table_count);

    const struct table_entry *dir = extmem_get_directory();
    if (!dir) {
        printk("Testing external tables: directory not available\n");
        return -EINVAL;
    }

    if (hdr->table_count > 0) {
        uint32_t tsize = dir[0].size;
        if (tsize == 0) {
            printk("Table0 size is zero\n");
            return -EINVAL;
        }
        if (tsize > MAX_TABLE_SIZE) {
            printk("Table0 size %u > MAX_TABLE_SIZE %u\n", (unsigned)tsize, (unsigned)MAX_TABLE_SIZE);
            return -EINVAL;
        }

        uint32_t table_flash_off = hdr->data_offset + dir[0].offset;
        int rc = flash_read_helper(table_flash_off, s_table_buf, tsize);
        if (rc) {
            return rc;
        }

        printk("Table0 size: %u\n", dir[0].size);
        printk("Table0 first byte: %02X\n", s_table_buf[0]);
    }

    return 0;
}
