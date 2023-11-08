
# Setting up Host PC environments

1. Install software
- Install terminator
```sudo apt install terminator```
- Install Docker
  - https://docs.docker.com/engine/install/ubuntu/
- Install VSCode, and Remote-Containers extension(recomended).
  - https://code.visualstudio.com/

2. Port forwarding
- Port forwarding is required to use real robots and sensors on docker container.
```sudo ufw allow 50000:50004```

3. Network setting
- Set static IP address
  ex) ```192.168.56.2```

# Setting up robot environments
## Common
- Set static IP address
  ex) ```192.168.56.3```

## Cobotta

## Universal Robot

# Quick setup of ROS environment on Docker container
1. Build docker image
```cd ./docker && ./BUILD-DOCKER-IMAGE.sh```



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
