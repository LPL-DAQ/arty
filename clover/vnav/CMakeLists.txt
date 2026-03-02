cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
set(CMAKE_CPP_COMPILER "${CROSS_COMPILE}g++")
project(vnav)

target_sources(app PRIVATE
    src/main.cpp
    src/vn_hal.cpp
)

target_include_directories(app PRIVATE
    src/
)