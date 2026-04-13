target_sources(app PRIVATE
    main.cpp
    server.cpp
    Controller.cpp
    Error.cpp
    FlightController.cpp
    LookupTable.cpp
    MutexGuard.cpp
    Trace.cpp

    hornet/RCSHornetActuator.cpp
    hornet/RCSHornetModule.cpp
    hornet/TVCHornetActuator.cpp
    hornet/TVCHornetModule.cpp
    hornet/ThrottleHornetActuator.cpp
    hornet/ThrottleHornetModule.cpp

    ranger/RCSRangerActuator.cpp
    ranger/RCSRangerModule.cpp
    ranger/TVCRangerActuator.cpp
    ranger/TVCRangerModule.cpp
    ranger/ThrottleRangerActuator.cpp
    ranger/ThrottleRangerModule.cpp
    ranger/ThrottleValve.cpp
)

add_subdirectory(sensors)
