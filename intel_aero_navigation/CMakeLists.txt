cmake_minimum_required(VERSION 2.8.3)
project(intel_aero_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  geometry_msgs
  genmsg
  actionlib_msgs
  actionlib
  nav_msgs
  grid_mapping
  nodelet
)

add_action_files(
  FILES
  WaypointNavigation.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_navigation waypoint_navigation_nodelet
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    genmsg
    actionlib_msgs
    actionlib
    nav_msgs
    grid_mapping
    nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(waypoint_navigation src/waypoint_navigation.cpp)
add_dependencies(waypoint_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(waypoint_navigation ${catkin_LIBRARIES})

add_library(waypoint_navigation_nodelet src/waypoint_navigation_nodelet.cpp)
target_link_libraries(waypoint_navigation_nodelet waypoint_navigation ${catkin_LIBRARIES})

add_executable(waypoint_navigation_node src/waypoint_navigation_node.cpp)
target_link_libraries(waypoint_navigation_node waypoint_navigation ${catkin_LIBRARIES})