# This is needed so that custom driver classes using system calls are taken into
# account

zephyr_include_directories(include)

add_subdirectory(drivers)
