cmake_minimum_required(VERSION 3.13.1)
set(CONFIG_MCUX_COMPONENT_driver.lpspi ON)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# For god's sake, why isn't this set automatically?
set(CMAKE_CPP_COMPILER "${CROSS_COMPILE}g++")

message(STATUS "C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "C++ compiler: ${CMAKE_CPP_COMPILER}")

project(app)

# Add nanopb dependency
list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)

zephyr_include_directories(src)
zephyr_nanopb_sources(app ../api/clover.proto)

add_subdirectory(src)

# Ranger 1 shares its GPIO output engine with the named-sequence client.
if(BOARD STREQUAL "ranger_1/mimxrt1062" OR BOARD STREQUAL "ranger_1")
    set(valve_dir "${CMAKE_CURRENT_SOURCE_DIR}/src/sequencing")
    # Migrate the former default cache entries; preserve custom paths.
    get_filename_component(old_valve_dir "${CMAKE_CURRENT_SOURCE_DIR}/../tests/valve_gpio" ABSOLUTE)
    foreach(entry VALVE_SEQUENCE_DIR VALVE_MAP_FILE)
        if(DEFINED ${entry})
            get_filename_component(old_value "${${entry}}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
            if((entry STREQUAL "VALVE_SEQUENCE_DIR" AND old_value STREQUAL "${old_valve_dir}/sequences") OR
               (entry STREQUAL "VALVE_MAP_FILE" AND old_value STREQUAL "${old_valve_dir}/valve_map.json"))
                unset(${entry} CACHE)
            endif()
        endif()
    endforeach()
    set(VALVE_SEQUENCE_DIR "${valve_dir}/sequences" CACHE PATH "Named sequence directory")
    set(VALVE_MAP_FILE "${valve_dir}/valve_map.json" CACHE FILEPATH "Valve mapping")
    if(VALVE_SEQUENCE_FILE)
        get_filename_component(sequence_input "${VALVE_SEQUENCE_FILE}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
        set(sequence_files "${sequence_input}")
    else()
        get_filename_component(sequence_input "${VALVE_SEQUENCE_DIR}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
        file(GLOB sequence_files CONFIGURE_DEPENDS "${sequence_input}/*.json")
    endif()
    get_filename_component(VALVE_MAP_FILE "${VALVE_MAP_FILE}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    set(sequence_header "${CMAKE_CURRENT_BINARY_DIR}/generated/valve_sequence.h")
    add_custom_command(OUTPUT "${sequence_header}"
        COMMAND ${PYTHON_EXECUTABLE} "${valve_dir}/generate_sequence.py"
                "${sequence_input}" "${sequence_header}" --valve-map "${VALVE_MAP_FILE}"
        DEPENDS ${sequence_files} "${VALVE_MAP_FILE}" "${valve_dir}/generate_sequence.py" "${valve_dir}/run_sequence.py"
        VERBATIM)
    target_sources(app PRIVATE "${valve_dir}/embedded_sequence.c" "${valve_dir}/sequence_protocol.c" "${sequence_header}")
    target_include_directories(app PRIVATE "${valve_dir}" "${CMAKE_CURRENT_BINARY_DIR}/generated")
    target_compile_definitions(app PRIVATE CLOVER_SEQUENCE_INTEGRATION)
endif()
