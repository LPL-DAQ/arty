find_package(nanopb REQUIRED)

# Generate C code from your proto in the api folder
nanopb_generate_cpp(proto_sources proto_headers ../api/clover.proto RELPATH ../api)

target_sources(app PRIVATE 
    main.cpp 
    server.cpp 
    pts.cpp 
    sequencer.cpp 
    ThrottleValve.cpp 
    Error.cpp 
    Controller.cpp 
    Trace.cpp
    ${proto_sources}
)

# Ensure headers are found in the build directory
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(app PRIVATE nanopb)
add_subdirectory(guards)