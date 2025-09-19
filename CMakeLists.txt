cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(footpod-firmware)

target_sources(app PRIVATE
    src/main.c
    src/ux.c
    src/imu_service.c
)

set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/nrf5340dk_nrf5340_cpuapp.overlay")
set(mcuboot_CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/child_image/mcuboot.conf)