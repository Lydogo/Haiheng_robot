cmake_minimum_required(VERSION 3.1.3)
project(Haiheng_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_ros_warehouse
  moveit_visual_tools
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  std_msgs
  control_msgs
  controller_manager_msgs
  actionlib
  actionlib_msgs
  roscpp
  rospy
  std_srvs
  message_generation
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()
add_executable(action_server src/action_server.cpp)
target_link_libraries(action_server ${catkin_LIBRARIES})
add_dependencies(action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
