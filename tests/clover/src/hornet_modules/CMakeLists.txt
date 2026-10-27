# HornetThrottle test expectations are stale and are expected to fail.
target_sources(app PRIVATE
    HornetRCS_test.cpp
    HornetTVC_test.cpp
    HornetThrottle_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/hornet/HornetRcs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/hornet/HornetTvc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/hornet/HornetThrottle.cpp
)
