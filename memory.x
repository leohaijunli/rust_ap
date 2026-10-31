/* STM32H753II: original PX4 bootloader owns flash sector 0. */
MEMORY
{
    FLASH      (rx)  : ORIGIN = 0x08020000, LENGTH = 1920K
    RAM        (rwx) : ORIGIN = 0x24000000, LENGTH = 447K
    FAULTSTACK (rw)  : ORIGIN = 0x2406FC00, LENGTH = 1K
    STACK      (rw)  : ORIGIN = 0x24070000, LENGTH = 64K
}

_stack_start = ORIGIN(STACK) + LENGTH(STACK);
_stack_end = ORIGIN(STACK);
__fault_stack_bottom = ORIGIN(FAULTSTACK);
__fault_stack_top = ORIGIN(FAULTSTACK) + LENGTH(FAULTSTACK);

/* No heap; cortex-m-rt 0.7.7 paints _stack_end.._stack_start. */
__sheap = _stack_end;
ASSERT(__euninit <= ORIGIN(FAULTSTACK), "static RAM overlaps the fault stack");
ASSERT(_stack_start == 0x24080000, "unexpected initial stack pointer");
