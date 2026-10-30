/* STM32F407IG. Sector 11 (0x080E0000..0x08100000) is reserved for the flash lesson. */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 896K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
