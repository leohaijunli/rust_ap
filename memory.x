/* STM32H753II: application after the 128 KiB USB bootloader, AXI SRAM for data. */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08020000, LENGTH = 1920K
    RAM   (rwx) : ORIGIN = 0x24000000, LENGTH = 512K
}
