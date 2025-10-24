# Dashboard/CMakeLists.txt

# at repo root CMake
if (NOT PICO_NO_HARDWARE)
    add_subdirectory(dashboard/wifi)
endif ()
