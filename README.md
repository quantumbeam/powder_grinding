
This is powder grinding package for ROS.
This package is used the papers for [Robotic Powder Grinding with a Soft Jig for Laboratory Automation in Material Science](https://doi.org/10.1109/IROS47612.2022.9981081) in IROS 2022 and [Robotic Powder Grinding with Audio-Visual Feedback for Laboratory Automation in Materials Science](https://omron-sinicx.github.io/powder-grinding/) in IROS 2023.

TOC
- [Overview](#overview)
- [Supported robot](#supported-robot)
- [Quick start](#quick-start)
  - [Setting up environments of Host PC, robot and docker](#setting-up-environments-of-host-pc-robot-and-docker)
  - [Running docker container](#running-docker-container)
  - [Build ROS packages on Docker container](#build-ros-packages-on-docker-container)
- [Known Issues](#known-issues)
- [Tuture work](#tuture-work)
- [License](#license)


# Overview
Update: 2023/10/24
This repository is for ROS environment for robot control.

# Supported robot
- UR5e
- UR3e (confirmed motion only simulation)
- Cobotta

# Quick start

## Setting up environments of Host PC, robot and docker
[Read setup document](./docker/README.md)


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

# Tuture work
- [ ] Automated estimation of mortar X,Y position by camera (mortar position detection by camera)
    - Camera jig and calibration is required
- [ ] Automated estimation of mortar Z position by force sensor


# License
This repository is licensed under the MIT license, see [LICENSE](./LICENSE).
