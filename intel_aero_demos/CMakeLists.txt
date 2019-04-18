cmake_minimum_required(VERSION 2.8.3)
project(intel_aero_demos)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  actionlib
  intel_aero_navigation
  roscpp
  geometry_msgs
  mavros_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES intel_aero_demos
  CATKIN_DEPENDS
    actionlib
    roscpp
    geometry_msgs
    mavros_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(patrol_demo src/patrol.cpp)
target_link_libraries(patrol_demo ${catkin_LIBRARIES})
