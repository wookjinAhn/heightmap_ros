cmake_minimum_required(VERSION 3.0.2)
project(heightmap_ros)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

set(heightmap_ros_src
        src/Boundary.cpp
        src/HeightmapNode.cpp
        src/Point3.cpp
        src/Point2.cpp
        src/QuadtreeNode.cpp
)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        )
find_package(camel-euclid-vector REQUIRED)

catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(heightmap_ros example/topic.cpp ${heightmap_ros_src})
target_link_libraries(heightmap_ros ${catkin_LIBRARIES} camel-euclid-vector )