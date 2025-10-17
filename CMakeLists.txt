# cmake_minimum_required(VERSION 3.12)

# # Pull in SDK (must be before project)
# include(pico_sdk_import.cmake)
# include(pico_extras_import_optional.cmake)
# include(FreeRTOS_Kernel_import.cmake) 

# project(INF2004_AAI10 C CXX ASM) 
# set(CMAKE_C_STANDARD 11) 
# set(CMAKE_CXX_STANDARD 17) 
# set(PICO_BOARD pico_w) 

# if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
#     message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
# endif()

# # Initialize the SDK
# pico_sdk_init()


# add_compile_options(-Wall
#         -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
#         -Wno-unused-function # we have some for the docs that aren't called
#         )
# if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
#     add_compile_options(-Wno-maybe-uninitialized)
# endif()

# # Add the directory that contains FreeRTOSConfig.h 
# include_directories(${CMAKE_SOURCE_DIR})  # Add root directory to include path 

# if (NOT PICO_NO_HARDWARE) 
#     add_subdirectory(car)
# endif ()


cmake_minimum_required(VERSION 3.13)

# Pull in Pico SDK (sets PICO_SDK_PATH from env or CMake cache)
include(pico_sdk_import.cmake)

project(INF2004_AAI10 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

<<<<<<< Updated upstream
# Build the firmware in car/
add_subdirectory(car)
=======

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

# Add the directory that contains FreeRTOSConfig.h 
include_directories(${CMAKE_SOURCE_DIR})  # Add root directory to include path 

if (NOT PICO_NO_HARDWARE) 
    add_subdirectory(car)
endif ()
>>>>>>> Stashed changes
