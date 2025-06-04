# ベースイメージとしてROS Noeticのdesktop-fullバージョンを使用
FROM osrf/ros:noetic-desktop-full

# メンテナー情報を設定
LABEL maintainer="YusakuNakajima<yusaku_nakajima@ap.eng.osaka-u.ac.jp>"

# キーボードレイアウト設定を無視するための環境変数
ENV DEBIAN_FRONTEND=noninteractive

# タイムゾーンを東京に設定
RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime

# ROSリポジトリを信頼済みとして追加(ROS1がEOLになった後でもインストールできるようにするための処理、本来は非推奨)
# 既存の ros1-latest.list を削除（競合防止）
RUN rm -f /etc/apt/sources.list.d/ros1-latest.list
# 必要なツールをインストール
RUN apt-get update \
    && apt-get install -y curl gnupg lsb-release \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
# ROSキーリングディレクトリの作成
RUN mkdir -p /etc/apt/keyrings
# ROS公開鍵をダウンロードし、新しいキーリングファイルとして保存
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /etc/apt/keyrings/ros-archive-keyring.gpg
# ROSリポジトリをsources.list.dに追加
# focal 環境であることを明示的に指定し、新しいキーリングファイルを使用します。
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" | tee /etc/apt/sources.list.d/ros.list > /dev/null

# gitとtzdataをインストール
RUN apt-get update \
    && apt-get install -y git tzdata \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 現在の日付を表示 (ビルド時の情報)
RUN date

# 言語環境を英語 (US) UTF-8に設定
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8

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
    libxmlrpcpp-dev \
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
    ros-noetic-rosparam-shortcuts \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-moveit \
    libxmlrpcpp-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install vcstools
RUN apt-get update && apt-get install -y \
    python3-vcstool \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ESMの設定
# 環境変数としてESMトークンを渡す (ビルド時)
ARG UBUNTU_PRO_TOKEN
# ESMクライアントのインストールとESMの有効化
# ここでは pro attach を実行して、イメージビルド時にESMを有効化
# apt dist-upgrade もここで実行して、イメージに最新のESMアップデートを含める
# pro enable esm-infra および pro enable esm-apps は自動で有効化されるため、明示的に指定する必要はありません。
RUN apt update && \
    apt install -y ubuntu-pro-client && \
    pro attach $UBUNTU_PRO_TOKEN && \
    pro enable ros && \
    apt dist-upgrade -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

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


