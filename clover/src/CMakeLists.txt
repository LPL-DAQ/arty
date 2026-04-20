target_sources(app PRIVATE
    main.cpp
    psram_test.c
)

# if(CONFIG_HORNET)
#     message(NOTICE "Building with Hornet sources")
#     add_subdirectory(hornet)
# elseif(CONFIG_RANGER)
#     message(NOTICE "Building with Ranger sources")
#     add_subdirectory(ranger)
# else()
#     message(FATAL_ERROR "Either CONFIG_HORNET or CONFIG_RANGER must be specified.")
# endif()
