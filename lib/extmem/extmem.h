#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= */
/* External Tables Blob Layout   */
/* ============================= */

/* Base address of QSPI memory-mapped region (only valid if memory-mapped mode enabled) */
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
    uint32_t directory_offset;  /* Offset to table directory (from start of flash blob) */
    uint32_t directory_size;    /* Size of directory in bytes */
    uint32_t data_offset;       /* Offset to start of table data region (from start of flash blob) */
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
/* API: Flash-based (safe) access */
/* ============================= */

/*
 * Initialize extmem subsystem (optional).
 * - Should ensure flash device is ready.
 * - Returns 0 on success or negative errno.
 */
int extmem_init(void);

/*
 * Read the tables header from external flash into caller-provided buffer.
 * - out_hdr: pointer to caller-provided struct tables_header
 * - Returns 0 on success, negative errno on failure.
 *
 * Note: this reads from flash (flash_read) and does not assume memory-mapped access.
 */
int extmem_read_header(struct tables_header *out_hdr);

/*
 * Read the directory entries from external flash.
 * - out_dir: caller-provided buffer to receive directory entries
 * - max_entries: capacity (in number of struct table_entry) of out_dir
 * - hdr: pointer to a valid tables_header (read via extmem_read_header)
 * - On success, returns number of directory entries read (>=0).
 * - Returns negative errno on failure.
 */
int extmem_read_directory(struct table_entry *out_dir, size_t max_entries,
                          const struct tables_header *hdr);

/*
 * Read an entire table by index into a caller buffer.
 * - index: 0..hdr->table_count-1
 * - out_buf: buffer to receive table data
 * - buf_len: length of out_buf in bytes (must be >= table size)
 * - hdr and dir must be provided (read via extmem_read_header / extmem_read_directory)
 * - Returns 0 on success, negative errno on failure (or -ENOSPC if buf too small).
 *
 * This is a safe flash_read-based API (no pointer into EXTMEM_BASE).
 */
int extmem_read_table_by_index(uint32_t index,
                               uint8_t *out_buf, size_t buf_len,
                               const struct tables_header *hdr,
                               const struct table_entry *dir);

/*
 * Read an entire table by table ID into a caller buffer.
 * - id: the table id to search for in the directory
 * - out_buf/out_len same semantics as extmem_read_table_by_index
 * - Returns 0 on success, negative errno on failure.
 */
int extmem_read_table_by_id(uint32_t id,
                            uint8_t *out_buf, size_t buf_len,
                            const struct tables_header *hdr,
                            const struct table_entry *dir);

/*
 * Read a range (offset+length) inside a table (useful for streaming/partial reads).
 * - table_base_offset: absolute offset of table data region start (hdr->data_offset)
 * - table_rel_offset: offset inside the table to begin reading (i.e. dir[i].offset + ... )
 * - out_buf/buf_len: buffer and requested length
 * - Returns 0 on success, negative errno on failure.
 */
int extmem_read_table_range(uint32_t table_base_offset,
                            uint32_t table_rel_offset,
                            uint8_t *out_buf, size_t buf_len);

/* ============================= */
/* Legacy / Memory-mapped helper */
/* ============================= */

/*
 * extmem_enable_mmap():
 * - Optional helper to enable memory-mapped (XIP) mode for the QSPI device so that
 *   EXTMEM_BASE can be safely dereferenced.
 * - On platforms where this is implemented, this function must initialize the
 *   QSPI controller and put it into memory-mapped mode.
 * - Returns 0 on success, negative errno on failure.
 *
 * WARNING: If you do not implement memory-mapped mode (or the device driver does
 * not provide it), do NOT dereference pointers at EXTMEM_BASE — that will cause
 * a UsageFault. Prefer the flash_read-based APIs above.
 */
int extmem_init(void);

/* ============================= */
/* Write / Downloader APIs      */
/* ============================= */

/*
 * Write a test pattern into external flash (uses flash_erase + flash_write).
 * Kept as-is from your previous implementation.
 */
int extmem_write_test_pattern(void);

/*
 * Write an arbitrary chunk into external flash.
 * - data: pointer to data in RAM
 * - size: length in bytes
 * - flash_offset: offset in flash blob to write to (0-based)
 * Returns 0 on success.
 */
int extmem_write_chunk(const uint8_t *data,
                       size_t size,
                       uint32_t flash_offset);

/*
 * Optional test routine that validates table contents (flash-read based).
 * The original extmem_test_tables() that dereferenced EXTMEM_BASE must be
 * reimplemented to use extmem_read_header / extmem_read_directory / extmem_read_table_xxx.
 */
int extmem_test_tables(void);

/* Keep original test table symbol (if you still want a RAM-based test table) */
extern const uint8_t extmem_test_table[1024];

/* ============================= */
/* Downloader (ST-LINK Debug Mode) */
/* ============================= */

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
