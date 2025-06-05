#!/bin/bash

### Ros Workspace ###
# Set up the workspace
sudo apt update


# Updating ROSDEP and installing dependencies
rosdep update --include-eol-distros
vcs import src < src/powder_grinding/.rosinstall
vcs pull src
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

# Build
catkin build

# Update enviromental veriables
source /opt/ros/$ROS_DISTRO/setup.bash
source devel/setup.bash
