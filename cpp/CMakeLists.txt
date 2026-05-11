cmake_minimum_required(VERSION 3.10)
project(yoloseg_laser_infer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(yoloseg_laser_infer yoloseg_laser_infer.cpp)
target_include_directories(yoloseg_laser_infer PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(yoloseg_laser_infer PRIVATE ${OpenCV_LIBS})
