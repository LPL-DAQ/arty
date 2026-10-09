target_sources(app PRIVATE
    RangerThrottle_test.cpp
    Sequence_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/ranger/RangerThrottle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/ranger/Sequence.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/MutexGuard.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/Error.cpp
)
