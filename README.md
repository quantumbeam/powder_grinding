
## Overview
Update: 2023/10/24
This repository is for ROS environment for robot control.

# Supported robot
- UR5e
- UR3e (confirmed motion only simulation)
- Cobotta

# Quick start

## Setting up environments of Host PC, robot and docker
[Setup document](./docker/README.md)


## Running docker container
- Run docker container
```./RUN-DOCKER-CONTAINER.sh```

- Run docker container and launch terminals on terminator
```./LAUNCH-TERMINATOR-TERMINAL.sh```

## Build ROS packages on Docker container
- Run build command in catkin_ws
```./BUILD_ROS_WORKSPACE.sh```






# Known Issues
- .dea file of Cobotta description is not readable, although it is recomended by DENSO.
  - Use .stl file instead of .dea file.
- 

# TODO
- Automated estimation of mortar positon 
  - X,Y coordinate setting by camera (mortar position detection)
    - Need to camera jig
    - Need to set camera position and orientation (Camera calibration)
  - Z coordinate setting by force sensor
    - Need to create estimation Z coodinate algorithm
- 


