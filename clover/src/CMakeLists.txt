target_sources(app PRIVATE main.cpp
    server.cpp
    Controller.cpp
    FlightController.cpp
    Error.cpp
    Trace.cpp
    LookupTable.cpp
    MutexGuard.cpp)

add_subdirectory(sensors)
add_subdirectory(rcs)
add_subdirectory(throttle)
add_subdirectory(tvc)
add_subdirectory(flight)
