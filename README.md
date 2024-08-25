### Robotic Powder Grinding for Laboratory Automation
<img src="https://github.com/quantumbeam/powder_grinding/blob/main/wiki/grinding_demo.gif?raw=true" alt="UR powder grinding" width="500">

Custum ROS packages for robotic powder grinding.
This package can operate both in simulation (Gazebo) and on the actual robot.


#### **Table of Contents**
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

## Supported Robots
- Confirmed to work with actual machines
   - UR5e (Universal Robot)
   - UR3e (Universal Robot)
   - Cobotta (DENSO WAVE)
   - FR3 (FAIRINO)
- Confirmed to work with simulation only
   - Cobotta PRO 900 (DENSO WAVE)

### Quick Start Guide
You can also view the Japanese version of the [README_jp](./README_jp.md).

#### Setting up Environments of Host PC, Robot, and Docker
- [Setup Instructions](./env/docker/README.md)
- [Setup Instructions (Japanese version)](./env/docker/README_jp.md)

#### Running Docker Container
- Runing docker container on terminal: `cd ./env && ./RUN-DOCKER-CONTAINER.sh`
- Launch Terminator and running docker container: `cd ./env && ./LAUNCH-TERMINATOR-TERMINAL.sh`

#### Building ROS Packages in Docker Container
- Execute only once on first `./INITIAL_SETUP_ROS_ENVIROMENTS.sh` in `catkin_ws` in docker conatner.  
- Execute to build `./BUILD_ROS_WORKSPACE.sh` in `catkin_ws` in docker conatner.


#### Demonstration
- We have prepared demo files for launching and performing grinding motions.
- Robot Launch:
   ```
   roslaunch grinding_robot_bringup ur5e_bringup.launch
   roslaunch grinding_robot_bringup ur3e_bringup.launch
   roslaunch grinding_robot_bringup cobotta_bringup.launch
   roslaunch grinding_robot_bringup cobotta_pro_900_bringup.launch
   roslaunch grinding_robot_bringup fr3_bringup.launch

   ```
  - If you want to use simulation, please launch with sim:=true.
- Launching Grinding Motion:
   ```
   roslaunch grinding_motion_routines ur3e_grinding_demo.launch
   roslaunch grinding_motion_routines ur5e_grinding_demo.launch
   roslaunch grinding_motion_routines cobotta_grinding_demo.launch
   roslaunch grinding_motion_routines cobotta_pro_900_grinding_demo.launch
   roslaunch grinding_motion_routines fr3_grinding_demo.launch

   ```
   - Use the command g to prepare for grinding (g=grinding), and then use y to execute the grinding.
   - Use the command G to prepare for powder collection with a spatula (G=grinding), and then use y to execute the powder collection.
- Grinding Parameters Configuration:
   - The configuration settings are located in the config directory within the grinding_motion_routines package.

### Known Issues
- Cobotta's .dea file is unreadable (use fixed .dae file from cobotta_description_converter.py in grinding_descriptions pkg).


### Future Work
- Add IKFast for motion planning
 - Need to load custom URDF for grinding on IKFast
- Automated calibration of mortar position using a force sensor.

### Citation
- [Robotic Powder Grinding with a Soft Jig for Laboratory Automation in Material Science](https://doi.org/10.1109/IROS47612.2022.9981081) (IROS 2022)
```
@InProceedings{RoboticPowderGrindingWithSoftJig,
  Title                    = {Robotic Powder Grinding with a Soft Jig for Laboratory Automation in Material Science},
  Author                   = {Yusaku Nakajima, Masashi Hamaya, Yuta Suzuki, Takafumi Hawai, Felix Von Drigalski, Kazutoshi Tanaka, Yoshitaka Ushiku and Kanta Ono.},
  Booktitle                = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  Year                     = {2022},
  Doi                      = {10.1109/IROS47612.2022.9981081}
}
```
If you are interested in the application of robotic powder grinding, please take a look at...
- [Robotic Powder Grinding with Audio-Visual Feedback for Laboratory Automation in Materials Science](https://ieeexplore.ieee.org/document/10341526) (IROS 2023)
   -  Github pages [here](https://omron-sinicx.github.io/powder-grinding/) 

### License
This repository is under the MIT license. See [LICENSE](./LICENSE) for details.

