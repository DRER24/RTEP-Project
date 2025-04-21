cmake_minimum_required(VERSION 3.10)
project(RadarTracking VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

# Set source files
set(SOURCES
    main.cpp
    Target.cpp
    RadarData.cpp
    KalmanFilter.cpp
    ServoController.cpp
    SerialComm.cpp
    SignalHandler.cpp
    RadarSystem.cpp
)

# Set header files
set(HEADERS
    Target.h
    RadarData.h
    KalmanFilter.h
    ServoController.h
    SerialComm.h
    SignalHandler.h
    RadarSystem.h
)

# Create executable
add_executable(radar_tracking ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(radar_tracking PRIVATE include)

# Link libraries
target_link_libraries(radar_tracking PRIVATE pthread)

# Install target
install(TARGETS radar_tracking DESTINATION bin)

# Add tests (if enabled)
option(BUILD_TESTS "Build tests" OFF)

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
