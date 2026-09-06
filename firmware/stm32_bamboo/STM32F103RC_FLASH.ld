/* Linker script for STM32F103RCT6 : 256K Flash, 48K RAM.
 * Bare-metal (frameworkless) build using the project's own CMSIS/StdPeriph.
 * Defines the symbols required by startup_stm32f103xe.s:
 *   _sidata _sdata _edata _sbss _ebss _estack
 *
 * IAP LAYOUT : the first 16 KB of Flash hold the resident IAP bootloader
 * (separate project in bootloader/). This application is relocated to
 * 0x08004000 and its vector table is pointed there via -DVECT_TAB_OFFSET=0x4000
 * (see SystemInit). The top 32 bytes of RAM are reserved for the shared
 * app<->bootloader flag (see lib/IAP/iap_protocol.h) and removed from RAM here.
 */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08004000, LENGTH = 240K
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 48K - 32
}

ENTRY(Reset_Handler)

/* Minimum heap/stack kept for the C library / FreeRTOS static allocations. */
_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text*)
        KEEP(*(.init))
        KEEP(*(.fini))

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.rodata*)

        KEEP(*(.eh_frame*))
        . = ALIGN(4);
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(vtable)
        *(.data*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))
        . = ALIGN(4);
        _edata = .;
    } > RAM AT> FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > RAM

    /* Not cleared by startup (outside _sbss.._ebss): survives a soft reset
     * (NVIC_SystemReset). Holds the "jump to ROM bootloader" marker. */
    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        *(.noinit*)
        . = ALIGN(4);
    } > RAM

    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > RAM

    /* Top of RAM used as initial stack pointer by the vector table. */
    _estack = ORIGIN(RAM) + LENGTH(RAM);

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
