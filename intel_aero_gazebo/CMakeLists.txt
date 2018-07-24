cmake_minimum_required(VERSION 2.8.3)
project(intel_aero_gazebo)

cmake_policy(SET CMP0046 NEW)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  octomap_server
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    mavros_msgs
    octomap_server
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(offboard_test src/offboard_test.cpp)
target_link_libraries(offboard_test ${catkin_LIBRARIES})

add_executable(navigation_test src/navigation_test.cpp)
target_link_libraries(navigation_test ${catkin_LIBRARIES})
