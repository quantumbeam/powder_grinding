### Robotic Powder Grinding for Laboratory Automation
<img src="https://github.com/quantumbeam/powder_grinding/blob/main/wiki/grinding_demo.gif?raw=true" alt="UR powder grinding" width="500">

Custum ROS packages for robotic powder grinding.
This package can operate both in simulation (fake joint and Rviz) and on the actual robot.
And provide Docker containers for the ROS environment.

### **Table of Contents**
- [Supported Robots](#supported-robots)
- [Quick Start Guide](#quick-start-guide)
  - [Soft-Jig](#soft-jig)
  - [Setting up Environments of Host PC, Robot, and Docker](#setting-up-environments-of-host-pc-robot-and-docker)
  - [Running Docker Container](#running-docker-container)
  - [Building ROS Packages in Docker Container](#building-ros-packages-in-docker-container)
  - [Demonstration](#demonstration)
- [Known Issues](#known-issues)
- [Release Memo](#release-memo)
- [Future Work](#future-work)
- [Citation](#citation)
- [License](#license)


## Supported Robots
- Confirmed to work with actual machines
   - UR5e (Universal Robot)
   - UR3e (Universal Robot)
   - Cobotta (DENSO WAVE)
   - FR3 (FAIRINO)
- Confirmed to work with simulation only
   - Cobotta PRO 900 (DENSO WAVE)

## Quick Start Guide
You can also view the Japanese version of the [README_jp](./README_jp.md).

### Soft-Jig
Please read below.
- [How to make Soft-Jig](./grinding_descriptions/mesh/3D_print_jig/README.md)
-  [How to make Soft-Jig (Japanese version)](./grinding_descriptions/mesh/3D_print_jig/README_jp.md)
### Setting up Environments of Host PC, Robot, and Docker
Please read below.
- [Setup Instructions](./env/docker/README.md)
- [Setup Instructions (Japanese version)](./env/docker/README_jp.md)

### Running Docker Container
- Runing docker container on terminal: `cd ./env && ./RUN-DOCKER-CONTAINER.sh`
- Launch Terminator and running docker container: `cd ./env && ./LAUNCH-TERMINATOR-TERMINAL.sh`

### Building ROS Packages in Docker Container
- Execute only once on first `./INITIAL_SETUP_ROS_ENVIROMENTS.sh` in `catkin_ws` in docker conatner.  
- Execute to build `./BUILD_ROS_WORKSPACE.sh` in `catkin_ws` in docker conatner.


### Demonstration
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
   - Use the command `g` to prepare for grinding (g=grinding), and then use `y` to execute the grinding.
   - Use the command `G` to prepare for powder collection with a spatula (G=grinding), and then use `y` to execute the powder collection.
- Grinding Parameters Configuration:
   - The configuration settings are located in the config directory within the grinding_motion_routines package.

## Known Issues
- Cobotta's .dea file is unreadable for Rviz (use fixed .dae file from cobotta_description_converter.py in grinding_descriptions pkg).

## Release Memo
- v1.0.0 (2025/02/01)
   - Initial release.
- v2.0.0 (2025/xx/xx)
   - Change the simulation software from Gazebo to Fake Joint on Rviz.
     - Gazebo is too much for the simulation of position control robots.
   - Update re-planning algorithm for grinding motion.
     - Total planning time is reduced more than 50% for many cases.
   - Add automated calibration of mortar position using a force sensor (supported only UR3e/UR5e).
     - The calibration is performed by the robot itself.
   - 


## Future Work
- Automated calibration of mortar position using a force sensor.

## Citation
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

## License
This repository is under the MIT license. See [LICENSE](./LICENSE) for details.

