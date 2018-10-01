cmake_minimum_required(VERSION 2.8.3)
project(network_status)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(network_visualization_node src/network_visualization_node.cpp)
target_link_libraries(network_visualization_node ${catkin_LIBRARIES})
