cmake_minimum_required(VERSION 2.8.3)
project(intel_aero_experiments)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  geometry_msgs
  nav_msgs
  actionlib
  intel_aero_navigation
  network_planner
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    actionlib
    intel_aero_navigation
    network_planner
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

add_executable(outdoor_test src/outdoor_test.cpp)
target_link_libraries(outdoor_test ${catkin_LIBRARIES})

add_executable(outdoor_action_test src/outdoor_action_test.cpp)
target_link_libraries(outdoor_action_test ${catkin_LIBRARIES})
