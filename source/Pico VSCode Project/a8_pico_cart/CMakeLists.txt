cmake_minimum_required(VERSION 3.13)

# DEVE vir ANTES do include do SDK
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")
set(PICO_BOARD pico2 CACHE STRING "Board")

include(pico_sdk_import.cmake)

project(a8_pico_cart)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(a8_pico_cart)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(a8_pico_cart)

# WeAct RP2350B flash size:
#   8MB  = 8388608  (0x800000)  — currently available
#   16MB = 16777216 (0x1000000) — when available
target_compile_definitions(a8_pico_cart PRIVATE
    PICO_FLASH_SIZE_BYTES=8388608
)

target_sources(a8_pico_cart PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/main.c
    ${CMAKE_CURRENT_LIST_DIR}/atari_cart.c
    ${CMAKE_CURRENT_LIST_DIR}/msc_disk.c
    ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/fatfs_disk.c
    ${CMAKE_CURRENT_LIST_DIR}/flash_fs.c
    ${CMAKE_CURRENT_LIST_DIR}/fatfs/ff.c
    ${CMAKE_CURRENT_LIST_DIR}/fatfs/ffunicode.c
    ${CMAKE_CURRENT_LIST_DIR}/fatfs/diskio.c
)

target_include_directories(a8_pico_cart PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/fatfs
)

# pico_stdlib + hardware_flash + tinyusb_device
# hardware_sync included for __wfe() used in timing loops
target_link_libraries(a8_pico_cart PUBLIC
    pico_stdlib
    hardware_flash
    hardware_sync
    tinyusb_device
)

# create map/bin/hex/uf2 file in addition to ELF
pico_add_extra_outputs(a8_pico_cart)
