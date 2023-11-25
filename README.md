### Powder Grinding Package for ROS

**Repository for Robotic Powder Grinding in Material Science**

#### **Related Publications**
- [Robotic Powder Grinding with a Soft Jig for Laboratory Automation in Material Science](https://doi.org/10.1109/IROS47612.2022.9981081) (IROS 2022)
- [Robotic Powder Grinding with Audio-Visual Feedback for Laboratory Automation in Materials Science](https://omron-sinicx.github.io/powder-grinding/) (IROS 2023)

#### **Table of Contents**
1. [Overview](#overview)
2. [Supported Robots](#supported-robot)
3. [Quick Start Guide](#quick-start)
   - [Environment Setup](#setting-up-environments-of-host-pc-robot-and-docker)
   - [Running Docker Container](#running-docker-container)
   - [Building ROS Packages](#build-ros-packages-on-docker-container)
4. [Known Issues](#known-issues)
5. [Future Work](#tuture-work)
6. [License](#license)

### Overview
**Last Updated:** 2023/10/24  
This repository focuses on the ROS environment for robot control.

### Supported Robots
- UR5e
- UR3e (confirmed in simulation only)
- Cobotta

### Quick Start Guide

#### Setting up Environments of Host PC, Robot, and Docker
- [Setup Instructions](./docker/README.md)

#### Running Docker Container
- Execute: `./RUN-DOCKER-CONTAINER.sh`
- With Terminator: `./LAUNCH-TERMINATOR-TERMINAL.sh`

#### Building ROS Packages in Docker Container
- Inside `catkin_ws`: `./BUILD_ROS_WORKSPACE.sh`

### Known Issues
- Cobotta's .dea file is unreadable (use fixed .dae file from cobotta_description_converter.py in grinding_descriptions pkg).
- Build failure in **moveit_calibration_plugins** due to missing `numpy/arrayobject.h`. Solution: Add the numpy path to `CMakeLists.txt`'s `include_directories`.
  - Example path: `/usr/local/lib/python3.8/dist-packages/numpy/core/include`

### Future Work
- Automated detection of mortar X,Y positions using a camera.
- Automated detection of mortar Z position using a force sensor.

### License
This repository is under the MIT license. See [LICENSE](./LICENSE) for details.
