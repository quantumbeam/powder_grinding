
# Setting up Host PC environments

1. Install software
- Install terminator
  - ```sudo apt install terminator```
- Install Docker
  - https://docs.docker.com/engine/install/ubuntu/
- Install VSCode, and Remote-Containers extension(recomended).
  - https://code.visualstudio.com/

2. Port forwarding
- Port forwarding is required to use real robots and sensors on docker container.
  - UR ports.
    - ```sudo ufw allow 50001```
    - ```sudo ufw allow 50002```
    - ```sudo ufw allow 50003```
    - ```sudo ufw allow 50004```
  - Cobotta port.
    - ```sudo ufw allow 5007```

3. Network setting
- Set static IP address on LAN adaptor
  - This package default : ```192.168.56.5```

# Setting up robot environments
## Common
- Set static IP address on robot LAN adaptor
  - This package default of UR :  ```192.168.56.42```
  - This package default of cobotta : ```192.168.56.11```

## Cobotta

## Universal Robot
- Install ```external control.urcap``` and setting LAN info on UR tablet
  - https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases

# Build docker image of ROS environment on Docker container
- Build docker image.
  - ```cd ./env/docker && ./BUILD-DOCKER-IMAGE.sh```
- You can run docker container, see "Running Docker Container" on [README.md](../README.md).



# Enviroment information
## Host PC
- Ubuntu: 20.04
  - ROS Noetic
- Docker 24.0.6
  - > 24.0.0 is required to use script "RUN-DOCKER-CONTAINER.sh"
  - Docker-compose is not required, used compose subcommand in docker.

## Robot
**Cobotta**
  - Software version: 2.16.12
  - MCU version: 
**UR5e**
  - Software version: 5.11.6
