cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr COMPONENTS unittest REQUIRED HINTS $ENV{ZEPHYR_BASE})

# For god's sake, why isn't this set automatically?
set(CMAKE_CPP_COMPILER "${CROSS_COMPILE}g++")

message(STATUS "C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "C++ compiler: ${CMAKE_CPP_COMPILER}")

project(app)

# Add nanopb dependency
list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)


zephyr_include_directories(${CMAKE_CURRENT_BINARY_DIR})
zephyr_nanopb_sources(app ../api/clover.proto)


target_sources(app PRIVATE src/main.cpp src/server.cpp src/pts.cpp src/sequencer.cpp src/ThrottleValve.cpp src/Error.cpp src/Controller.cpp src/Trace.cpp)

# Sources with unit tests
target_sources(testbinary PRIVATE src/Trace.cpp)
