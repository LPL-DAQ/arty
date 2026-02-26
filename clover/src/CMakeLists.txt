add_subdirectory(guards)

target_sources(app PRIVATE main.cpp)
target_sources(app PRIVATE server.cpp)
target_sources(app PRIVATE pts.cpp)
target_sources(app PRIVATE ThrottleValve.cpp)
target_sources(app PRIVATE Controller.cpp)
target_sources(app PRIVATE Error.cpp)
target_sources(app PRIVATE Trace.cpp)
