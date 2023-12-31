### Robotic Powder Grinding for Laboratory Automation
<img src="https://github.com/quantumbeam/powder_grinding/blob/main/wiki/grinding_demo.gif?raw=true" alt="UR powder grinding" width="500">

Custum ROS packages for robotic powder grinding.
This package can operate both in simulation (Gazebo) and on the actual robot.


#### **Table of Contents**
- [Robotic Powder Grinding for Laboratory Automation](#robotic-powder-grinding-for-laboratory-automation)
  - [**Table of Contents**](#table-of-contents)
- [Overview](#overview)
- [Supported Robots](#supported-robots)
- [Quick Start Guide](#quick-start-guide)
  - [Setting up Environments of Host PC, Robot, and Docker](#setting-up-environments-of-host-pc-robot-and-docker)
  - [Running Docker Container](#running-docker-container)
  - [Building ROS Packages in Docker Container](#building-ros-packages-in-docker-container)
  - [Demonstration](#demonstration)
- [Known Issues](#known-issues)
- [Future Work](#future-work)
- [Citation](#citation)
- [License](#license)

### Overview
**Last Updated:** 2023/10/24  
This repository focuses on the ROS environment for robot control.



### Supported Robots
- UR5e
- UR3e
- Cobotta

### Quick Start Guide

#### Setting up Environments of Host PC, Robot, and Docker
- [Setup Instructions](./docker/README.md)

#### Running Docker Container
- Runing docker container on terminal: `./RUN-DOCKER-CONTAINER.sh`
- Launch Terminator and running docker container: `./LAUNCH-TERMINATOR-TERMINAL.sh`

#### Building ROS Packages in Docker Container
- Execute only once on first `./INITIAL_SETUP_ROS_ENVIROMENTS.sh` in `catkin_ws` on docker conatner.  
- Execute `./BUILD_ROS_WORKSPACE.sh` in `catkin_ws` on docker conatner.


#### Demonstration
- We have prepared demo files for launching and performing grinding motions.
- Robot Launch:
   ```
   roslaunch grinding_robot_bringup ur5e_bringup.launch
   roslaunch grinding_robot_bringup ur3e_bringup.launch
   roslaunch grinding_robot_bringup cobotta_bringup.launch
   ```
  - If you want to use simulation, please launch with sim:=true.
- Launching Grinding Motion:
   ```
   roslaunch grinding_motion_routines ur5e_grinding_demo.launch
   ```
   - Use the command g to prepare for grinding (g=grinding), and then use y to execute the grinding.
   - Use the command G to prepare for powder collection with a spatula (G=grinding), and then use y to execute the powder collection.
- Grinding Parameters Configuration:
   - The configuration settings are located in the config directory within the grinding_motion_routines package.

### Known Issues
- Cobotta's .dea file is unreadable (use fixed .dae file from cobotta_description_converter.py in grinding_descriptions pkg).

### Future Work
- Automated calibration of mortar position using a force sensor.

### Citation
- [Robotic Powder Grinding with a Soft Jig for Laboratory Automation in Material Science](https://doi.org/10.1109/IROS47612.2022.9981081) (IROS 2022)
```
@InProceedings{RoboticPowderGrinding,
  Title                    = {Robotic Powder Grinding with a Soft Jig for Laboratory Automation in Material Science},
  Author                   = {Yusaku Nakajima, Masashi Hamaya, Yuta Suzuki, Takafumi Hawai, Felix Von Drigalski, Kazutoshi Tanaka, Yoshitaka Ushiku and Kanta Ono.},
  Booktitle                = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  Year                     = {2022},
  Doi                      = {10.1109/IROS47612.2022.9981081}
}
- [Robotic Powder Grinding with Audio-Visual Feedback for Laboratory Automation in Materials Science](https://omron-sinicx.github.io/powder-grinding/) (IROS 2023)
```
- Related article: [Robotic Powder Grinding with Audio-Visual Feedback for Laboratory Automation in Materials Science](https://omron-sinicx.github.io/powder-grinding/) (IROS 2023)

### License
This repository is under the MIT license. See [LICENSE](./LICENSE) for details.

