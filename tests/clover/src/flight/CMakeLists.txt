target_sources(app PRIVATE
    FlightController_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/flight/FlightController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/Trace.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/Error.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/MutexGuard.cpp)
