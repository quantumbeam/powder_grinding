
FROM osrf/ros:noetic-desktop-full
LABEL maintainer="Nakajima<yusaku_nakajima@ap.eng.osaka-u.ac.jp>"

# Ignore keybourd layout seting
ENV DEBIAN_FRONTEND=noninteractive

# set timezone
RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime
RUN apt-get update \
    && apt-get install -y git tzdata

RUN date
# Install apt packages
RUN apt-get update && apt-get install -y \
    git \
    vim \
    nano \
    wget \
    tmux \
    curl \
    terminator \
    iputils-ping net-tools \
    libglib2.0-0 \
    libgl1-mesa-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-joy ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
    ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python ros-noetic-rosserial-client \
    ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
    ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
    ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
    ros-noetic-interactive-markers \
    ros-noetic-dynamixel-sdk \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install vcstools
RUN apt-get update && apt-get install -y \
    python3-vcstool \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup catkin workspace
RUN mkdir -p ~/catkin_ws/src
COPY BUILD_ROS_WORKSPACE.sh /root/catkin_ws
RUN chmod +x ~/catkin_ws/BUILD_ROS_WORKSPACE.sh
COPY INITIAL_SETUP_ROS_ENVIROMENTS.sh /root/catkin_ws
RUN chmod +x ~/catkin_ws/INITIAL_SETUP_ROS_ENVIROMENTS.sh

# Create ROS workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc


