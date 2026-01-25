add_subdirectory(guards)

target_sources(app PRIVATE main.cpp server.cpp pts.cpp sequencer.cpp ThrottleValve.cpp Error.cpp Controller.cpp Trace.cpp)
