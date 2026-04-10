target_sources(app PRIVATE
    ThrottleController.cpp
    ThrottleStateIdle.cpp
    ThrottleStateThrustSeq.cpp
    ThrottleStateAbort.cpp
)

add_subdirectory(ranger)
add_subdirectory(hornet)
