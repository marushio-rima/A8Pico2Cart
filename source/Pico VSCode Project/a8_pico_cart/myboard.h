// myboard.h — WeAct RP2350B
// Not used when PICO_BOARD=pico2 in CMakeLists.txt
// Kept for reference only

// WeAct RP2350B uses W25Q64 (8MB) or W25Q128 (16MB) flash
// Both are compatible with default PICO_FLASH_SPI_CLKDIV=2
// #define PICO_FLASH_SPI_CLKDIV 2

// Pick up settings from pico2 board definition
#include "boards/pico2.h"
