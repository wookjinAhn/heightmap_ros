cmake_minimum_required(VERSION 3.0.2)
project(heightmap_ros)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        )

catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(heightmap_ros src/topic.cpp include/heightmap.h src/heightmap.cpp)
target_link_libraries(heightmap_ros ${catkin_LIBRARIES})