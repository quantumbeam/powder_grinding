cmake_minimum_required(VERSION 3.0.2)
project(grinding_robot_control)


# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  tf
  tf2_ros
)

catkin_package(
 CATKIN_DEPENDS rospy roscpp std_msgs tf tf2_ros
)
