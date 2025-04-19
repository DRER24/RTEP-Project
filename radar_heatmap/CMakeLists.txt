cmake_minimum_required(VERSION 3.10)
project(RadarHeatMapManager VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Add library
add_library(RadarHeatMapManager
    src/heatmap_manager.cpp
)

# You'll need to add your radar_detector implementation
# Example:
# add_library(RadarDetector
#     src/radar_detector.cpp
# )

# Create an example application (uncomment when you have a main.cpp)
# add_executable(HeatMapExample src/main.cpp)
# target_link_libraries(HeatMapExample PRIVATE RadarHeatMapManager)

# Install
install(TARGETS RadarHeatMapManager
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES 
    include/heatmap_manager.h
    DESTINATION include
)
