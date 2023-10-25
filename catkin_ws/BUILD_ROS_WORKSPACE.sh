### Ros Workspace ###
# Set up the workspace
sudo apt update

rosdep update

# Updating ROSDEP and installing dependencies
cd /home/ubuntu/onolab/catkin_ws && vcs import src < src/.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

# Build
catkin build

# Update enviromental veriables
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ubuntu/onolab/catkin_ws/devel/setup.bash
