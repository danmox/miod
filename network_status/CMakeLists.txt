cmake_minimum_required(VERSION 2.8.3)
project(network_status)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  actionlib
  actionlib_msgs
  roscpp
  message_generation
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  channel_simulator
  intel_aero_navigation
  routing_msgs
)

add_message_files(
  FILES
  RatePair.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    roscpp
    std_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    channel_simulator
    intel_aero_navigation
    message_runtime
    routing_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(channel_visualization_node src/channel_visualization_node.cpp)
add_dependencies(channel_visualization_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(channel_visualization_node ${catkin_LIBRARIES})

add_executable(routing_visualization_node src/routing_visualization_node.cpp)
add_dependencies(routing_visualization_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(routing_visualization_node ${catkin_LIBRARIES})

add_executable(routing_channel_visualization_node src/routing_channel_visualization_node.cpp)
add_dependencies(routing_channel_visualization_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(routing_channel_visualization_node ${catkin_LIBRARIES})

add_executable(intel_demo src/intel_demo.cpp)
target_link_libraries(intel_demo ${catkin_LIBRARIES})
