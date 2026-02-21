# Generate Protobuf headers and sources
# PROJECT_SOURCE_DIR is 'clover/'. We go up one level to 'arty/' then into 'api/'
zephyr_nanopb_sources(app ${PROJECT_SOURCE_DIR}/../api/clover.proto)

# Required so the compiler can find clover.pb.h in the build folder
zephyr_include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Map all your source files into the build
target_sources(app PRIVATE
    main.cpp
    server.cpp
    pts.cpp
    sequencer.cpp
    ThrottleValve.cpp
    Controller.cpp
    Error.cpp
    Trace.cpp
)

# Allow headers in 'src' to find each other
target_include_directories(app PRIVATE .)
