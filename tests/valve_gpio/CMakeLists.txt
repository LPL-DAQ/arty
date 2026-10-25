cmake_minimum_required(VERSION 3.20.0)
set(CONFIG_MCUX_COMPONENT_driver.lpspi ON)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(valve_gpio_test)
if(CONFIG_VALVE_TEST_EMBEDDED_SEQUENCE AND CONFIG_VALVE_TEST_AUTOMATIC)
    message(FATAL_ERROR "Select embedded sequence or automatic single-channel test, not both")
endif()
if(CONFIG_VALVE_TEST_EMBEDDED_SEQUENCE)
    set(VALVE_SEQUENCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/sequence.example.json" CACHE FILEPATH "JSON valve sequence")
    get_filename_component(VALVE_SEQUENCE_FILE "${VALVE_SEQUENCE_FILE}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    set(sequence_header "${CMAKE_CURRENT_BINARY_DIR}/generated/valve_sequence.h")
    add_custom_command(
        OUTPUT "${sequence_header}"
        COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/generate_sequence.py"
                "${VALVE_SEQUENCE_FILE}" "${sequence_header}"
        DEPENDS "${VALVE_SEQUENCE_FILE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/generate_sequence.py"
                "${CMAKE_CURRENT_SOURCE_DIR}/run_sequence.py"
        VERBATIM)
    target_sources(app PRIVATE src/embedded_sequence.c "${sequence_header}")
    target_include_directories(app PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
elseif(CONFIG_VALVE_TEST_AUTOMATIC)
    target_sources(app PRIVATE src/automatic.c)
else()
    target_sources(app PRIVATE src/main.c)
endif()
