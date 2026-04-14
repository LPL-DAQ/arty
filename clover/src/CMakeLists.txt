target_sources(app PRIVATE main.cpp
    server.cpp
    ThrottleValve.cpp
    lidar.cpp
    Controller.cpp
    Error.cpp
    Trace.cpp
    LookupTable.cpp
    StateIdle.cpp
    StateCalibrateValve.cpp
    StateValveSeq.cpp
    StateThrustSeq.cpp
    StateAbort.cpp
    AnalogSensors.cpp
    MutexGuard.cpp
    PwmActuator.cpp
    )
