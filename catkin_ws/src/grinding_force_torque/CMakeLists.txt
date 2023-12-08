cmake_minimum_required(VERSION 3.0.2)
project(grinding_force_torque)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  WrenchStatistics.srv
)



catkin_package(
 INCLUDE_DIRS include
 LIBRARIES leptrino
 CATKIN_DEPENDS rospy roscpp std_msgs

)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

# C++ files
add_executable(leptrino_force_torque
  src/leptrino_force_torque.cpp
  src/rs_comm.cpp
)

target_link_libraries(leptrino_force_torque
  ${catkin_LIBRARIES}
)

