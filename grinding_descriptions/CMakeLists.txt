cmake_minimum_required(VERSION 3.0.2)
project(grinding_descriptions)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

# Need to import the python modules from the package setup.py
catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
