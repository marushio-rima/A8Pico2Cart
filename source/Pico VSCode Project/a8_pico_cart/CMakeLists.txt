cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")
set(PICO_BOARD pico2 CACHE STRING "Board")

include(pico_sdk_import.cmake)

project(a8pico2cart)

pico_sdk_init()

add_executable(a8pico2cart)

target_compile_definitions(a8pico2cart PRIVATE
    PICO_FLASH_SIZE_BYTES=8388608
    USBD_PRODUCT="A8Pico2Cart"
)

target_sources(a8pico2cart PUBLIC
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

target_include_directories(a8pico2cart PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/fatfs
)

target_link_libraries(a8pico2cart PUBLIC
    pico_stdlib
    hardware_flash
    hardware_sync
    hardware_clocks
    tinyusb_device
)

pico_add_extra_outputs(a8pico2cart)
