cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

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

# -----------------------------------------------------------------------
# Lookup table code generation
# Drop a CSV in tables/ and a constexpr header will be generated into
# the build directory automatically. Include it as <stem_table.h>.
# e.g. tables/throttle.csv -> #include "throttle_table.h"
# -----------------------------------------------------------------------
find_program(PYTHON3 python3 REQUIRED)

file(GLOB LOOKUP_TABLE_CSVS "${CMAKE_CURRENT_SOURCE_DIR}/tables/*.csv")

set(GENERATED_LOOKUP_HEADERS "")

foreach(CSV_FILE ${LOOKUP_TABLE_CSVS})
    get_filename_component(CSV_NAME ${CSV_FILE} NAME_WE)
    set(OUT_HEADER "${CMAKE_CURRENT_BINARY_DIR}/${CSV_NAME}_table.h")

    add_custom_command(
        OUTPUT  ${OUT_HEADER}
        COMMAND ${PYTHON3}
                ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_lookup_table.py
                ${CSV_FILE}
                ${OUT_HEADER}
        DEPENDS ${CSV_FILE}
                ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_lookup_table.py
        COMMENT "Generating lookup table header from ${CSV_NAME}.csv"
        VERBATIM
    )

    list(APPEND GENERATED_LOOKUP_HEADERS ${OUT_HEADER})
    message(STATUS "Registered lookup table: ${CSV_NAME}.csv -> ${CSV_NAME}_table.h")
endforeach()

add_custom_target(lookup_tables_generated DEPENDS ${GENERATED_LOOKUP_HEADERS})
add_dependencies(app lookup_tables_generated)

add_subdirectory(src)
