### Ros Workspace ###
# Set up the workspace
sudo apt update
sudo apt upgrade -y

sudo pip install --upgrade pip

pip3 install -r src/requirements.txt

sh BUILD_ROS_WORKSPACE.sh
