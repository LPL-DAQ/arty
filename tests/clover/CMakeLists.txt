cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

message(STATUS "C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "C++ compiler: ${CMAKE_CPP_COMPILER}")

project(app)

# Add nanopb dependency
list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)

zephyr_include_directories(${CMAKE_CURRENT_BINARY_DIR})
zephyr_nanopb_sources(app ../../api/clover.proto)

target_include_directories(app PRIVATE ../../darwin/src)
target_include_directories(app PRIVATE ../../clover/src)
add_subdirectory(src)
