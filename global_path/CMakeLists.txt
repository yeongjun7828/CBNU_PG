cmake_minimum_required(VERSION 3.0.2)
project(global_path)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp sensor_msgs)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/simple_path_ctrack_node.cpp src/simple_path_ctrack.cpp include/simple_path_ctrack/simple_path_ctrack.h)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

