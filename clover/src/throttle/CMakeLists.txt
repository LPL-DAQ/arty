target_sources(app PRIVATE
    ThrottleController.cpp
    ThrottleStateIdle.cpp
    ThrottleStateThrustSeq.cpp
    ThrottleStateFlight.cpp
    ThrottleStateAbort.cpp
    hornet/ThrottleHornet.cpp
    ranger/ThrottleRanger.cpp
    ranger/ThrottleStateValveSeq.cpp
    ranger/ThrottleStateCalibrateValve.cpp
    ranger/ThrottleValve.cpp
)

