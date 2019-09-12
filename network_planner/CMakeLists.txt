cmake_minimum_required(VERSION 2.8.3)
project(network_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  actionlib
  channel_simulator
  geometry_msgs
  grid_mapping
  intel_aero_navigation
  nav_msgs
  roscpp
  routing_msgs
  socp
  visualization_msgs
)

find_package(Armadillo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib
    channel_simulator
    geometry_msgs
    grid_mapping
    intel_aero_navigation
    nav_msgs
    roscpp
    routing_msgs
    socp
    visualization_msgs
  DEPENDS
    armadillo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/network_planner.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/network_planner_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(network_planner_test src/network_planner_test.cpp)
target_link_libraries(network_planner_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
