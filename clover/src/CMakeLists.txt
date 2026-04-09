target_sources(app PRIVATE main.cpp
    server.cpp
    ThrottleValve.cpp
    Controller.cpp
    Error.cpp
    Trace.cpp
    LookupTable.cpp
    StateIdle.cpp
    StateCalibrateValve.cpp
    StateValveSeq.cpp
    StateThrustSeq.cpp
    StateAbort.cpp
    MutexGuard.cpp)

add_subdirectory(sensors)
add_subdirectory(rcs)
add_subdirectory(throttle)
add_subdirectory(tvc)
