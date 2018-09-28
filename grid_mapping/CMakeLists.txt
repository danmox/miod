cmake_minimum_required(VERSION 2.8.3)
project(grid_mapping)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  nodelet
  rosbag
  tf
)

find_package(OpenCV REQUIRED)

add_message_files(FILES
  OccupancyGrid.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    message_generation
    rosbag
    tf
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES
  src/grid_base.cpp
  src/point.cpp
  src/common.cpp
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(grid_test src/grid_test.cpp)
target_link_libraries(grid_test ${PROJECT_NAME} ${catkin_LIBRARIES}
  ${OpenCV_LIBS})
