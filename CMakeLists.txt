cmake_minimum_required(VERSION 3.16)
project(surveillance VERSION 2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags for Raspberry Pi optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=armv8-a+crc -mtune=cortex-a72")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -funroll-loops")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# Find packages
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GPIOD_INCLUDE_DIRS})

# Source files
set(SOURCES
    main.cpp
    logger.cpp
    gpio.cpp
    camera.cpp
    motion.cpp
    recorder.cpp
    stream.cpp
    alert.cpp
)

# Executable
add_executable(surveillance ${SOURCES})

# Link libraries
target_link_libraries(surveillance
    ${OpenCV_LIBS}
    ${GPIOD_LIBRARIES}
    pthread
)

# Install
install(TARGETS surveillance DESTINATION /usr/local/bin)
