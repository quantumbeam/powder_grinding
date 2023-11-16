cmake_minimum_required(VERSION 3.0.2)
project(grinding_vision)

# # Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_srvs
  visualization_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  PowderPos.srv
  MortarCenter.srv
  DiagonalPowderRadius.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
)

catkin_package()