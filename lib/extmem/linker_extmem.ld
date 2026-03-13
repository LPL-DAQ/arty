/*
 * linker_extmem.ld
 *
 * Optional EXTMEM linker section.
 *
 * This section allows placing compile-time data into external QSPI flash
 * using:
 *
 *     __attribute__((section(".ext_rodata")))
 *
 * Currently NOT used.
 *
 * External tables are written at runtime via:
 *     - downloader.c
 *     - extmem_write_chunk()
 *
 * This linker section is kept for future support of:
 *     - ST-LINK external loader (.stldr)
 *     - Compile-time external flash placement
 */


    .ext_rodata :
    {
        KEEP(*(.ext_rodata*))
    } > EXTMEM

