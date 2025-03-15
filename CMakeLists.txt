cmake_minimum_required(VERSION 3.0)

project(RadarSensor)
find_package(PCL REQUIRED)
target_link_libraries(work_cmake gpiod pthread RadarSensor ${PCL_LIBRARIES})

add_executable(RadarSensor work_cmake main.cpp work.cpp radar_sensor.cpp pcl_visualization.cpp fft_processor.cpp radar_sensor.h pcl_visualization.h fft_processor.h)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(/usr/include)
