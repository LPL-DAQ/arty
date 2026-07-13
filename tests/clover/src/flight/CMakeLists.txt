target_sources(app PRIVATE
    FlightController_test.cpp
    MathUtil_test.cpp
    StateEstimator_test.cpp
    ZAxisEkf_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/flight/FlightController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/flight/StateEstimator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/flight/ZAxisEkf.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/Trace.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/Error.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/MutexGuard.cpp)
