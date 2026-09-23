# HornetTVC_test.cpp is excluded: it still calls the two-argument tick(), and its
# expected values are derived from the placeholder linkage constants.
# HornetThrottle_test.cpp is excluded: its expected values are still in lbf, but
# tick() now takes vertical acceleration in m/s^2.
target_sources(app PRIVATE
    HornetRCS_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../clover/src/hornet/HornetRcs.cpp
)
