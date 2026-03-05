#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= */
/* Existing API                  */
/* ============================= */

int extmem_enable_mmap(void);

int extmem_write_test_pattern(void);

int extmem_write_chunk(const uint8_t *data,
                       size_t size,
                       uint32_t flash_offset);

int extmem_test_tables(void);

/* Keep original test table symbol */
extern const uint8_t extmem_test_table[1024];

/* ============================= */
/* External Tables Blob Layout   */
/* ============================= */

/* Base address of QSPI memory-mapped region */
#define EXTMEM_BASE 0x90000000

/* Magic value used to validate tables blob ("TBLE") */
#define TABLE_MAGIC 0x54424C45

/*
 * Header located at the beginning of external flash.
 * Describes the layout of the tables blob.
 */
struct tables_header {
    uint32_t magic;             /* Magic number for validation */
    uint32_t version;           /* Tables format version */
    uint32_t table_count;       /* Number of tables stored */
    uint32_t directory_offset;  /* Offset to table directory (from EXTMEM_BASE) */
    uint32_t directory_size;    /* Size of directory in bytes */
    uint32_t data_offset;       /* Offset to start of table data region */
};

/*
 * Directory entry describing a single table.
 */
struct table_entry {
    uint32_t id;      /* Unique table identifier */
    uint32_t offset;  /* Offset to table data (relative to data region) */
    uint32_t size;    /* Table size in bytes */
};

/* ============================= */
/* Table Access API              */
/* ============================= */

/* Return pointer to tables header (located at EXTMEM_BASE) */
const struct tables_header *extmem_get_header(void);

/* Return pointer to table directory */
const struct table_entry *extmem_get_directory(void);

/* Get table pointer by index (0 .. table_count-1) */
const uint8_t *extmem_get_table_by_index(uint32_t index);

/* Get table pointer by table ID (returns NULL if not found) */
const uint8_t *extmem_get_table_by_id(uint32_t id);

/* Downloader (ST-LINK Debug Mode) */

/* These variables live in RAM. Host writes them via GDB to trigger programming. */
extern volatile uint32_t downloader_buf;      /* RAM address where blob is stored */
extern volatile uint32_t downloader_size;     /* Size of blob in bytes */
extern volatile uint32_t downloader_flash_offset; /* Flash offset to write this chunk */
extern volatile uint32_t downloader_pending;  /* Host sets to 1 to request write */
extern volatile uint32_t downloader_status;   /* 0 idle, 1 busy, 2 done, 3 error */

/* Called periodically in main loop */
void downloader_task_check_and_run(void);

#ifdef __cplusplus
}
#endif
