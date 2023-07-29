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

### PTP demo

To run the PTP demo, use the following command to initialize the rviz visualization:
```
ros2 launch lwr_description display.launch.py use_gui:=false
```
and start the joint trajectory generator:
```
ros2 run trajectory_generator joint_sinusoidal_trajectory
```
and finally start the 'ptp_demo' node:
```
ros2 run lwr_motion_program ptp_demo
```

If you wish, you can also start the [**LWR forward kinematics solver**](https://github.com/EleganckaPorazka/lwr_forward_kinematics.git):
```
ros2 run lwr_forward_kinematics lwr_forward_kinematics
```

### LIN demo

To run the LIN demo, use the following command to initialize the rviz visualization:
```
ros2 launch lwr_description display.launch.py use_gui:=false
```
and start the Cartesian trajectory generator:
```
ros2 run trajectory_generator cartesian_sinusoidal_trajectory
```
and the forward kinematics solver:
```
ros2 run lwr_forward_kinematics lwr_forward_kinematics
```
and the inverse kinematics solver:
```
ros2 run inverse_kinematics inverse_kinematics_basic 
```
and set its parameters:
```
ros2 param set /inverse_kinematics_basic DOF '7'
ros2 param set /inverse_kinematics_basic dt '0.005'
ros2 param set /inverse_kinematics_basic '[100.0, 1.0]'
```
and finally start the 'lin_demo' node:
```
ros2 run lwr_motion_program lin_demo
```

Remember to set 'dt' to the same value in *lin_demo* and as a parameter of *inverse_kinematics_basic*.

## Notes

This code is also uploaded to [**my other repository**](https://gitlab.com/lwolinski/lwr_motion_program.git).
