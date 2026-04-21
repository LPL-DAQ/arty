target_sources(app PRIVATE main.cpp
    server.cpp
    Controller.cpp
    Error.cpp
    Trace.cpp
    LookupTable.cpp
    StateIdle.cpp
    # TODO: FIXUP
    # StateCalibrateValve.cpp
    StateValveSeq.cpp
    StateThrustSeq.cpp
    StateAbort.cpp
    AnalogSensors.cpp
    Lidar.cpp
    VectornavIMU.cpp
    MutexGuard.cpp)


if(CONFIG_HORNET)
    message(NOTICE "Building with Hornet sources")
    add_subdirectory(hornet)
elseif(CONFIG_RANGER)
    message(NOTICE "Building with Ranger sources")
    add_subdirectory(ranger)
else()
    message(FATAL_ERROR "Either CONFIG_HORNET or CONFIG_RANGER must be specified.")
endif()
