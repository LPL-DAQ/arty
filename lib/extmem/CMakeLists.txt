# lib/extmem/CMakeLists.txt

zephyr_sources(extmem.c)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

zephyr_linker_sources(SECTIONS linker_extmem.ld)
