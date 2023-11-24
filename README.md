# lwr_motion_program

<img src="https://img.shields.io/badge/ros--version-humble-green"/>  <img src="https://img.shields.io/badge/platform%20-Ubuntu%2022.04-orange"/>

## Description

A ROS2 package to program a motion of LWR 4+ manipulator.

A work in progress in early stages.

## Installation

It is recommended to use Ubuntu 22.04 with [**ROS 2 Humble**](https://docs.ros.org/en/humble/index.html).

### Required packages

[**lwr_description**](https://github.com/EleganckaPorazka/kuka_lwr4plus_urdf.git)

[**lwr_forward_kinematics**](https://github.com/EleganckaPorazka/lwr_forward_kinematics.git)

[**inverse_kinematics**](https://github.com/EleganckaPorazka/inverse_kinematics.git)

[**trajectory_generator**](https://github.com/EleganckaPorazka/trajectory_generator.git)

[**rrlib_interfaces**](https://github.com/EleganckaPorazka/rrlib_interfaces.git)

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
and call the PTP demo launcher:
```
ros2 launch lwr_motion_program ptp_demo.launch.py
```

Instead of the PTP demo launcher, you can start the joint trajectory generator:
```
ros2 run trajectory_generator joint_sinusoidal_trajectory
```
and finally start the 'ptp_demo' node:
```
ros2 run lwr_motion_program ptp_demo --ros_args -p dt:=X
```
where 'X' is the desired time step in seconds.

If you wish, you can also start the [**LWR forward kinematics solver**](https://github.com/EleganckaPorazka/lwr_forward_kinematics.git):
```
ros2 run lwr_forward_kinematics lwr_forward_kinematics
```

### LIN demo

To run the LIN demo, use the following command to initialize the rviz visualization:
```
ros2 launch lwr_description display.launch.py use_gui:=false
```
and call the LIN demo launcher:
```
ros2 launch lwr_motion_program lin_demo.launch.py
```
An example:
```
ros2 launch lwr_motion_program lin_demo.launch.py dt:=0.01 clik_gains:=[100.0,1.0] tool:=[0.1,0.1,0.1,0.0,0.0,0.0,1.0]
```
starts the LIN demo and sets the parameters 'dt' (time step), 'clik_gains' (position and orientation error gain for CLIK), and 'tool' (position and orientation in quaternions of the tool with regards to the last link's frame) to the given values.

Instead of the LIN demo launcher, you can start the Cartesian trajectory generator:
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
ros2 param set /inverse_kinematics_basic dt '0.01'
ros2 param set /inverse_kinematics_basic clik_gains '[100.0, 1.0]'
```
or just use the launcher for the inverse kinematics solver:
```
ros2 launch inverse_kinematics inv_kin.launch.py
```
and finally start the 'lin_demo' node:
```
ros2 run lwr_motion_program lin_demo --ros_args -p dt:=X
```
where 'X' is the desired time step in seconds. Remember to set the same value to the parameter 'dt' of *lin_demo* as the 'dt' parameter of *inverse_kinematics_basic*.

## Notes

TODO: The *lin_demo* seems to be working OK in some runs, and wrong in others. Perhaps sometimes there is some mismatch (loss of synchronization?) when reading joint states? Time to explore the real-time kernel?

TODO: multiple goals to define PTP and LIN combinations.

TODO: *ptp_demo* and *lin_demo* don't update 'dt' parameter from the outside (be it a command line or a launcher). Guess, I missed the parameter handling.

TODO: *lwr_forward* doesn't update 'tool' parameter when called like that:
```
ros2 run lwr_forward_kinematics lwr_forward_kinematics --ros_args -p tool:=[0.1,0.1,0.1,0.0,0.0,0.0,1.0]
``` 

This code is also uploaded to [**my other repository**](https://gitlab.com/lwolinski/lwr_motion_program.git).
