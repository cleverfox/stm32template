

MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
  rom (rx) : ORIGIN = 0x08000000, LENGTH = 128K
}
_estack = 0x20000800;
_sidata = 0;
SECTIONS
  {
    .  = 0x0;          /* From 0x00000000 */
    .text : {
    *(.isr_vector)      /* Vector table */
    *(.text)        /* Program code */
    *(.rodata)      /* Read only data */
    _etext = .;
    /* This is used by the startup in order to initialize the .data secion */
    _sidata = _etext;
    } >rom

    .  = 0x20000000;   /* From 0x20000000 */
    .data : {
    _sdata = .;
    *(.data)        /* Data memory */
    _edata = .;
    } >ram AT > rom

    .bss : {
    _sbss = .;
    *(.bss)         /* Zero-filled run time allocate data memory */
    _ebss = .;
    } >ram AT > rom
 }  
/*========== end of file ==========*/
