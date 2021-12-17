cmake_minimum_required(VERSION 2.8.3)
project(socp)

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
  CATKIN_DEPENDS ${RUN_DEPENDS}
)
