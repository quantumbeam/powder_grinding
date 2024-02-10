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


# Copy description files to DENSO ROS package
rosrun denso_robot_bringup install_robot_description.py ./src/powder_grinding/grinding_descriptions/urdf/cobotta/setup_files/cobotta_description
rosrun denso_robot_bringup update_joint_limits.py cobotta ./src/powder_grinding/grinding_descriptions/urdf/cobotta/setup_files/joint_limits.yaml
echo "It is fine if cobotta's setup_files does not exist even if you run this script more than once."

echp "If you can't use roslauch, please run the 'source devel/setup.bash' on the terminal."