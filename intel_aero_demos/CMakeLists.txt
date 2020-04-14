cmake_minimum_required(VERSION 2.8.3)
project(intel_aero_demos)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  actionlib
  intel_aero_navigation
  roscpp
  geometry_msgs
  mavros_msgs
  network_planner
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    roscpp
    geometry_msgs
    mavros_msgs
    network_planner
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(swarm_demo src/swarm.cpp)
target_link_libraries(swarm_demo ${catkin_LIBRARIES})
