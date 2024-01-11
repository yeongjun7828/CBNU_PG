#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.5)

project(linepoints_map_pkg)

# Add -O0 to remove optimizations when using gcc
#IF(CMAKE_COMPILER_IS_GNUCC)
#    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
#    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

set(CMAKE_BUILD_TYPE Debug) # Release
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread -Wall")

find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    pcl_ros
    image_transport # For FreeSpaceDetection
    sensor_msgs     # For FreeSpaceDetection
    cv_bridge       # For FreeSpaceDetection
    )

catkin_package(
    #  INCLUDE_DIRS include
    LIBRARIES linepoints_map_pkg
    CATKIN_DEPENDS roscpp std_msgs
    DEPENDS system_lib
    )


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CGAL_DIR}
    /usr/local/include
    /usr/include
    src

    )


link_directories(
    /usr/local/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    )

# Add OpenCV Libraries
link_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})


add_executable(linepoints_map_pkg_exe

    src/main.cpp
    src/UTM.cpp
    )


target_link_libraries(linepoints_map_pkg_exe ${catkin_LIBRARIES})

