cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(app)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../lib/extmem extmem_build)

target_sources(app PRIVATE
    src/main.cpp
)
