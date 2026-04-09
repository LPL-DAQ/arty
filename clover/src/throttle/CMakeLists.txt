target_sources(app PRIVATE
    ThrottleController.cpp
    ThrottleValve.cpp
    ThrottleStateIdle.cpp
    ThrottleStateCalibrateValve.cpp
    ThrottleStateValveSeq.cpp
    ThrottleStateThrustSeq.cpp
    ThrottleStateAbort.cpp
)
