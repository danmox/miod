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
  gazebo_msgs
  nodelet
  mavros_msgs
  message_filters
  robot_localization
  tf2
  tf2_ros
  tf2_geometry_msgs
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
  LIBRARIES waypoint_navigation px4_nav_nodelet gazebo_vel_nav_nodelet
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    genmsg
    actionlib_msgs
    actionlib
    nav_msgs
    grid_mapping
    gazebo_msgs
    nodelet
    mavros_msgs
    message_filters
    robot_localization
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LIB_FILES
  src/nav_base.cpp
  src/mavros_uav.cpp
  src/gazebo_vel_nav.cpp
  include/intel_aero_navigation/px4_nav.hpp
)

add_library(waypoint_navigation ${LIB_FILES})
add_dependencies(waypoint_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(waypoint_navigation ${catkin_LIBRARIES})

add_library(px4_nav_nodelet src/px4_nav_nodelet.cpp)
target_link_libraries(px4_nav_nodelet waypoint_navigation ${catkin_LIBRARIES})

add_executable(px4_nav_node src/px4_nav_node.cpp)
target_link_libraries(px4_nav_node waypoint_navigation ${catkin_LIBRARIES})

add_executable(action_test src/action_test.cpp)
target_link_libraries(action_test waypoint_navigation ${catkin_LIBRARIES})

add_library(gazebo_vel_nav_nodelet src/gazebo_vel_nav_nodelet.cpp)
target_link_libraries(gazebo_vel_nav_nodelet waypoint_navigation ${catkin_LIBRARIES})

add_executable(gazebo_vel_nav_node src/gazebo_vel_nav_node.cpp src/gazebo_vel_nav.cpp)
target_link_libraries(gazebo_vel_nav_node waypoint_navigation ${catkin_LIBRARIES})

add_executable(action_test_vel src/action_test_vel.cpp)
target_link_libraries(action_test_vel waypoint_navigation ${catkin_LIBRARIES})

add_executable(gps_tf_node src/gps_tf_node.cpp)
target_link_libraries(gps_tf_node ${catkin_LIBRARIES})
