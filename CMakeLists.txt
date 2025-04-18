cmake_minimum_required(VERSION 3.10)
project(mmwave_radar_heatmap)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pthread")

# Add the source files
set(SOURCES
    main.cpp
    radar_detector.cpp
    heatmap_manager.cpp
    visualizer.cpp
)

# Create executable
add_executable(radar_heatmap ${SOURCES})

# Link libraries
target_link_libraries(radar_heatmap pthread)

# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)