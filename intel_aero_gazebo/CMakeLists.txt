cmake_minimum_required(VERSION 2.8.3)
project(intel_aero_gazebo)

cmake_policy(SET CMP0046 NEW)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  tf
  tf2
  tf2_ros
)
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    mavros_msgs
    tf
    tf2
    tf2_ros
  DEPENDS
    gazebo
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
list(APPEND CMAKE_CXX_FLAGS ${GAZEBO_CXX_FLAGS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(offboard_test src/offboard_test.cpp)
target_link_libraries(offboard_test ${catkin_LIBRARIES})

add_executable(model_tf_publisher src/model_tf_publisher.cpp)
target_link_libraries(model_tf_publisher ${catkin_LIBRARIES})

add_library(velocity_control src/set_velocity_plugin.cpp)
target_link_libraries(velocity_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
