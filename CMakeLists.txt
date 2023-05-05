cmake_minimum_required(VERSION 3.0.2)
project(first_project)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf
)

add_message_files(
  FILES
  Odom.msg
)

add_service_files(
  FILES
  ResetOdom.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
 # CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_node src/odom_node.cpp)

add_dependencies(odom_node ${catkin_EXPORTED_TARGETS})
add_dependencies(odom_node first_project_generate_messages_cpp) ## add this line to load custom msg BEFORE compiling (otherwise error)

target_link_libraries(odom_node ${catkin_LIBRARIES})

