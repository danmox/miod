cmake_minimum_required(VERSION 2.8.3)
project(channel_simulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  geometry_msgs
  octomap_msgs
  roscpp
)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    octomap_msgs
    roscpp
  DEPENDS
    octomap
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/channel_simulator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(channel_simulator_test src/channel_simulator_test.cpp)
target_link_libraries(channel_simulator_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
