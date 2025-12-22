cmake_minimum_required(VERSION 3.23)

project(floyd)

add_subdirectory(src)

# Upon install, write binary to ~/clover/bin, which is in PATH.
install(TARGETS floyd RUNTIME DESTINATION /home/lpl/clover/bin)

# Boost
# Note: Non-header-only components must be explicitly added as separate components: https://www.boost.org/doc/libs/latest/more/getting_started/unix-variants.html#header-only-libraries
find_package(Boost REQUIRED)
target_include_directories(floyd PRIVATE ${Boost_INCLUDE_DIRS})

# Labjack
target_link_libraries(floyd LabJackM)
