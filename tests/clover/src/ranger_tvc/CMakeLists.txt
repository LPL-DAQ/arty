target_sources(app PRIVATE
    tvc_kinematics_test.cpp
    moteus_test.cpp
    tvc_control_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/ranger/tvc/tvc_kinematics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/ranger/tvc/moteus.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/ranger/tvc/tvc_control.cpp
)
