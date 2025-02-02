cmake_minimum_required(VERSION 3.0.2)
project(grinding_force_torque)
set(CMAKE_CXX_STANDARD 14)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

## Generate messages
# add_message_files()
add_service_files(  
  FILES
  WrenchStatistics.srv
)
# add_action_files()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS rospy roscpp std_msgs
)

# Include directories
# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

# C++ files
# add_executable(${PROJECT_NAME}
# )

# Linking
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

