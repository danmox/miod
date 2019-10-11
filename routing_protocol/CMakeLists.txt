cmake_minimum_required(VERSION 2.8.3)
project(routing_protocol)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  routing_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    routing_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rt_update src/rt_update.cpp)
add_dependencies(rt_update ${catkin_EXPORTED_TARGETS})
target_link_libraries(rt_update ${catkin_LIBRARIES})

add_executable(rt_update_sub src/rt_update_sub.cpp)
add_dependencies(rt_update_sub ${catkin_EXPORTED_TARGETS})
target_link_libraries(rt_update_sub ${catkin_LIBRARIES})
