cmake_minimum_required(VERSION 3.0.2)
project(grinding_motion_routines)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  PositionCalibrateVector.srv
)

generate_messages(
  DEPENDENCIES
  std_srvs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})