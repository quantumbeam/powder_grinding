#!/bin/bash

sudo apt update
sudo apt upgrade -y

sudo pip install --upgrade pip

# Installing pip packages
pip3 install -r src/powder_grinding/requirements.txt

# Updating ROSDEP and installing dependencies
rosdep update
vcs import src < src/powder_grinding/.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

# Build
catkin build

# Update enviromental veriables
source /opt/ros/$ROS_DISTRO/setup.bash
source devel/setup.bash
