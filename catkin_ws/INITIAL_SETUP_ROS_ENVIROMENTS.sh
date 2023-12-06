#!/bin/bash

sudo apt update
sudo apt upgrade -y

sudo pip install --upgrade pip

# Installing pip packages
pip3 install -r src/requirements.txt

# Updating ROSDEP and installing dependencies
rosdep update
vcs import src < src/.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

# Build
catkin build

# Update enviromental veriables
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ubuntu/onolab/catkin_ws/devel/setup.bash
