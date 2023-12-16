cmake_minimum_required(VERSION 3.0.2)
project(grinding_motion_routines)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
)

# Need to import the python modules from the package setup.py
catkin_python_setup()


## C++ ##
# Create catkin info
# catkin_package()

# Include directories
# include_directories(${catkin_INCLUDE_DIRS})

# Add c++ files
# add_executable()

# Add libraries
# target_link_libraries(
#   ${catkin_LIBRARIES}
# )



