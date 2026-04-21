target_sources(app PRIVATE
    main.cpp
    AnalogSensors.cpp
    Error.cpp
    clover.pb.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    /home/lpl/modules/lib/nanopb
)

# if(CONFIG_HORNET)
#     message(NOTICE "Building with Hornet sources")
#     add_subdirectory(hornet)
# elseif(CONFIG_RANGER)
#     message(NOTICE "Building with Ranger sources")
#     add_subdirectory(ranger)
# else()
#     message(FATAL_ERROR "Either CONFIG_HORNET or CONFIG_RANGER must be specified.")
# endif()
