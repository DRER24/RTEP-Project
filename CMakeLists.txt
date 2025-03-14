cmake_minimum_required(VERSION 3.0)

project(work)

target_link_libraries(work_cmake gpiod pthread)

add_executable(work_cmake main.cpp work.cpp)

