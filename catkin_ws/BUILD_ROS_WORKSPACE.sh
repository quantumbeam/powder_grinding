### Ros Workspace ###
# Set up the workspace
source /opt/ros/$ROS_DISTRO/setup.bash

sudo apt update

rosdep update

# Updating ROSDEP and installing dependencies
cd ~/onolab/catkin_ws && vcs import src < src/.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

# Build
catkin build
source /home/ubuntu/onolab/catkin_ws/devel/setup.bash
