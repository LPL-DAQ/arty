target_sources(app PRIVATE
    main.cpp
    server.cpp
    Controller.cpp
    Error.cpp
    MutexGuard.cpp
    Trace.cpp
    util.cpp
)

add_subdirectory(sensors)
add_subdirectory(flight)

if(CONFIG_BLINK_LED)
target_sources(app PRIVATE BlinkLed.cpp)
endif()

if(CONFIG_THROTTLE_VALVES)
target_sources(app PRIVATE ThrottleValve.cpp)
endif()

if(CONFIG_VALVES)
target_sources(app PRIVATE Valves.cpp)
endif()

if(CONFIG_HORNET)
    message(NOTICE "Building with Hornet sources")
    add_subdirectory(hornet)
elseif(CONFIG_RANGER)
    message(NOTICE "Building with Ranger sources")
    add_subdirectory(ranger)
else()
    message(FATAL_ERROR "Either CONFIG_HORNET or CONFIG_RANGER must be specified.")
endif()
