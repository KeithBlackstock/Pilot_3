/* Linker script for GD32F303CCT6: 256K flash, 48K SRAM. */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 48K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  .isr_vector :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  .text :
  {
    *(.text*)
    *(.rodata*)
  } > FLASH

  /* Fixed at ORIGIN(RAM) == 0x20000000 by being the first RAM section --
   * must match kTelemetryAddr in telemetry.hpp and
   * tools/pilot3_attitude_monitor.py. */
  .telemetry (NOLOAD) :
  {
    . = ALIGN(4);
    _stelemetry = .;
    *(.telemetry)
    . = ALIGN(4);
    _etelemetry = .;
  } > RAM

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT> FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM
}
