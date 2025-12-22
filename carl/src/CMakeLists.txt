add_subdirectory(guards)

target_sources(app PRIVATE main.cpp)
target_sources(app PRIVATE server.cpp)
target_sources(app PRIVATE pts.cpp)
target_sources(app PRIVATE throttle_valve.cpp)
target_sources(app PRIVATE sequencer.cpp)
