cmake_minimum_required(VERSION 2.8.3)
project(socp)

add_compile_options(-std=c++11)

set(PKG_DEPENDS
  std_msgs
  geometry_msgs
  rospy
)
set(BUILD_DEPENDS
  ${PKG_DEPENDS}
  message_generation
)
set(RUN_DEPENDS
  ${PKG_DEPENDS}
  message_runtime
)

find_package(catkin REQUIRED ${BUILD_DEPENDS})

find_package(Armadillo REQUIRED)
find_package(LAPACK REQUIRED)
SET(CBLAS_LIBRARIES "-lcblas")

add_message_files(FILES
  QoS.msg
  Flow.msg
)

add_service_files(FILES
  RobustRoutingSOCP.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
  LIBRARIES ${PROJECT_NAME} ${ARMADILLO_LIBRARIES} armadillo_socp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LAPACK_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  )

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/socp.c)
target_link_libraries(${PROJECT_NAME} ${LAPACK_LIBRARIES}  ${ARMADILLO_LIBRARIES} ${CBLAS_LIBRARIES})

add_library(armadillo_socp src/armadillo_socp.cc src/socp.c)
target_link_libraries(armadillo_socp ${LAPACK_LIBRARIES}  ${ARMADILLO_LIBRARIES} ${CBLAS_LIBRARIES})

add_executable(armadillo_example src/armadillo_example.cc)
target_link_libraries(armadillo_example armadillo_socp)

add_executable(radio_example src/radio_example.cc)
target_link_libraries(radio_example armadillo_socp)

add_executable(radio_example2 src/radio_example2.cc)
target_link_libraries(radio_example2 armadillo_socp)

