cmake_minimum_required(VERSION 3.20.0)
set(CONFIG_MCUX_COMPONENT_driver.lpspi ON)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(valve_gpio_test)
if(CONFIG_VALVE_TEST_EMBEDDED_SEQUENCE AND CONFIG_VALVE_TEST_AUTOMATIC)
    message(FATAL_ERROR "Select embedded sequence or automatic single-channel test, not both")
endif()
if(CONFIG_VALVE_TEST_EMBEDDED_SEQUENCE)
    set(VALVE_SEQUENCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sequences" CACHE PATH "Directory of named JSON sequences")
    # Preserve explicit single-file builds; the normal build embeds the directory.
    if(VALVE_SEQUENCE_FILE)
        get_filename_component(sequence_input "${VALVE_SEQUENCE_FILE}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
        set(sequence_files "${sequence_input}")
    else()
        get_filename_component(sequence_input "${VALVE_SEQUENCE_DIR}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
        file(GLOB sequence_files CONFIGURE_DEPENDS "${sequence_input}/*.json")
    endif()
    set(VALVE_MAP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/valve_map.json" CACHE FILEPATH "Valve-name to GPIO-channel mapping")
    get_filename_component(VALVE_MAP_FILE "${VALVE_MAP_FILE}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    set(sequence_header "${CMAKE_CURRENT_BINARY_DIR}/generated/valve_sequence.h")
    add_custom_command(
        OUTPUT "${sequence_header}"
        COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/generate_sequence.py"
                "${sequence_input}" "${sequence_header}" --valve-map "${VALVE_MAP_FILE}"
        DEPENDS ${sequence_files} "${VALVE_MAP_FILE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/generate_sequence.py"
                "${CMAKE_CURRENT_SOURCE_DIR}/run_sequence.py"
        VERBATIM)
    list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
    include(nanopb)
    zephyr_nanopb_sources(app ../../api/clover.proto)
    target_sources(app PRIVATE src/embedded_sequence.c src/sequence_server.c src/sequence_protocol.c "${sequence_header}")
    target_include_directories(app PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
elseif(CONFIG_VALVE_TEST_AUTOMATIC)
    target_sources(app PRIVATE src/automatic.c)
else()
    target_sources(app PRIVATE src/main.c)
endif()
