
## Overview
Update: 2023/10/24

## Environment
- Ubuntu 20.04
  - ROS Noetic
- Python 3.8.5
- Docker 24.0.6
  - > 24.0.0 is required
- Docker-compose 2.21.0

# Enviroment preparation

## Preparation on local PC
- Install terminator
```sudo apt install terminator```
- Install Docker
  - https://docs.docker.com/engine/install/ubuntu/
- Install VSCode, and Remote-Containers extension(recomended).
  - https://code.visualstudio.com/

## Port forwarding
- Port forwarding is required to use GUI on docker container.
```sudo ufw allow 50000:50004```

## Network setting
### Rocal PC
- Set static IP address
  ex) ```192.168.56.2```
### Robot
- Set static IP address
  ex) ```192.168.56.3```


## Build docker image and run docker container
```cd ./docker && docker build ./ -t onolab/ros-noetic```
```./RUN-DOCKER-CONTAINER.sh```

## Build ROS packages on Docker container
- Run build command in catkin_ws
```./BUILD_ROS_WORKSPACE.sh```

# Running ROS environment after environment preparation
- Run docker container and launch terminals on terminator
```./LAUNCH-TERMINATOR-TERMINAL.sh```
## Run Powder Grinding with UR5e
- Preset command is written in terminator for ur5e.
roslaunch ur_control ur5e_moveit.launch
roslaunch kek_moveit_config ur5e_moveit_planning_execution.launch
roslaunch kek_moveit_config moveit_rviz.launch
mechano_grinding_fast.launch



## Trouble shooting
- Docker compose and docker-compose
- 


