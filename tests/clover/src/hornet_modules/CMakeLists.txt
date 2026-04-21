target_sources(app PRIVATE
    HornetThrottle_test.cpp
    HornetRCS_test.cpp
    HornetTVC_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/hornet/HornetThrottle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/hornet/HornetRcs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/hornet/HornetTvc.cpp
)
