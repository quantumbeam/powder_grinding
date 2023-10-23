FROM osrf/ros:noetic-desktop-full
LABEL maintainer="Nakajima<yusaku_nakajima@ap.eng.osaka-u.ac.jp>"

# キーボード設定の選択を聞かれないようにする
ARG DEBIAN_FRONTEND=noninteractive

# Install apt packages
RUN apt-get update && apt-get install -y \
    git \
    vim \
    wget \
    tmux \
    terminator && \
    iputils-ping net-tools \
    apt-get clean && rm -rf /var/lib/apt/lists/*

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
    ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
    ros-noetic-dynamixel-sdk \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3 && \
    nano \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Create ROS workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /home/ubuntu/onolab/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc
RUN mkdir -p /home/ubuntu/onolab/catkin_ws/src
RUN cd /home/ubuntu/onolab/catkin_ws && /bin/bash -c "source /opt/ros/noetic/setup.sh; catkin build"


# RUN cd /home/ubuntu/onolab/catkin_ws/src && git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
# RUN cd /home/ubuntu/onolab/catkin_ws && /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build"

