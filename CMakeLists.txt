cmake_minimum_required(VERSION 3.0.2)
project(single_height_pacakge)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

set(heightmap_ros_src
        src/Boundary.cpp
        src/HeightmapNode.cpp
        src/Point3.cpp
        src/Point2.cpp
        src/QuadtreeNode.cpp
)

set(heightmap_ros_old_src
        src/Boundary.cpp
        src/HeightmapNode_old.cpp

        src/Point3.cpp
        src/Point2.cpp
        src/QuadtreeNode_old.cpp
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

add_executable(single_height_pacakge example/usingTopic_old.cpp ${heightmap_ros_old_src})
target_link_libraries(single_height_pacakge ${catkin_LIBRARIES} camel-euclid-vector )