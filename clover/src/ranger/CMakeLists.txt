
target_sources(app PRIVATE
    RangerThrottle.cpp
    RangerRcs.cpp
    )

if(CONFIG_RANGER_TVC)
    target_sources(app PRIVATE
        RangerTvc.cpp
        tvc/tvc_kinematics.cpp
        tvc/tvc_control.cpp
        tvc/moteus.cpp
        tvc/moteus_can.cpp
        )
endif()
