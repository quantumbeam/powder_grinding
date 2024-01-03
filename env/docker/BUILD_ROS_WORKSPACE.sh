#!/bin/bash

### Ros Workspace ###
# Set up the workspace
sudo apt update

rosdep update

# Updating ROSDEP and installing dependencies
vcs import src < src/powder_grinding/.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
vcs pull src

# Build
catkin build

# Update enviromental veriables
source /opt/ros/$ROS_DISTRO/setup.bash
source devel/setup.bash
