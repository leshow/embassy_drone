MEMORY {
    /*
     * 2 MiB placeholder - this "mini zero" clone's actual flash chip is unconfirmed.
     * Verify against the board before anything here needs more than 2 MiB.
     */
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
    /*
     * RP2350 SRAM: 8 banks (SRAM0-7) with a striped mapping, 512K total.
     */
    RAM : ORIGIN = 0x20000000, LENGTH = 512K
    /*
     * Two extra dedicated (non-striped) 4K banks - 520K SRAM total, matching the datasheet.
     */
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
}

SECTIONS {
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH
} INSERT AFTER .vector_table;

/* explicit ALIGN(8) here - .start_block's size isn't naturally 8-byte aligned, which
   otherwise misaligns .text and trips rust-lld's alignment warning */
_stext = ALIGN(ADDR(.start_block) + SIZEOF(.start_block), 8);

SECTIONS {
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
    } > FLASH
} INSERT AFTER .uninit;

PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);
