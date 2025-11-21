
#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
set(DTC_OVERLAY_FILE
        $ENV{ZEPHYR_BASE}/../modules/lib/Arduino-Zephyr-API/variants/${BOARD}/${BOARD}.overlay
)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# For god's sake, why isn't this set automatically?
set(CMAKE_CPP_COMPILER "${CROSS_COMPILE}g++")

message(STATUS "C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "C++ compiler: ${CMAKE_CPP_COMPILER}")



project(app)

# Make Adafruit libraries' headers visible to main.cpp
target_include_directories(app PRIVATE
        $ENV{ZEPHYR_BASE}/../modules/lib/Arduino-Zephyr-API/libraries/Adafruit_BNO08x/src
        $ENV{ZEPHYR_BASE}/../modules/lib/Arduino-Zephyr-API/libraries/Adafruit_BusIO
        $ENV{ZEPHYR_BASE}/../modules/lib/Arduino-Zephyr-API/libraries/Adafruit_Sensor
        $ENV{ZEPHYR_BASE}/../modules/lib/Arduino-Zephyr-API/libraries/SPI
)
add_subdirectory(src)

