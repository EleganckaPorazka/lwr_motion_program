# lwr_motion_program

<img src="https://img.shields.io/badge/ros--version-humble-green"/>  <img src="https://img.shields.io/badge/platform%20-Ubuntu%2022.04-orange"/>

## Description

A ROS2 package to program a motion of LWR 4+ manipulator.

A work in progress in early stages.

## Installation

It is recommended to use Ubuntu 22.04 with [**ROS 2 Humble**](https://docs.ros.org/en/humble/index.html).

### Required packages

[**lwr_description**](https://github.com/EleganckaPorazka/kuka_lwr4plus_urdf.git)

[**trajectory_generator**](https://github.com/EleganckaPorazka/trajectory_generator.git)

### Building from source

```
mkdir -p ~/ros2_ws/src/lwr_motion_program
cd ~/ros2_ws/src/lwr_motion_program
git clone https://github.com/EleganckaPorazka/lwr_motion_program .
source /opt/ros/humble/setup.bash
colcon build
. install/setup.bash
```

## Running

To run the 'ptp_demo' node, use the following command:
```
ros2 run lwr_motion_program ptp_demo
```
