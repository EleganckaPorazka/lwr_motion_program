import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    # time step dt
    dt_launch_arg = DeclareLaunchArgument(
        "dt", default_value=TextSubstitution(text="0.01")
    )
    
    # end effector position (in meters) and orientation (in quaternions), relative to the last link's frame: [x,y,z,qx,qy,qz,qw]
    tool_launch_arg = DeclareLaunchArgument(
        "tool", default_value=TextSubstitution(text="[0.0,0.0,0.0,0.0,0.0,0.0,1.0]")
    )
    
    joint_trajectory_node = Node(
        package='trajectory_generator',
        executable='joint_sinusoidal_trajectory',
        namespace='',
        name='joint_sinusoidal_trajectory'
        )
    
    lwr_forward_kinematics_node = Node(
        package='lwr_forward_kinematics',
        executable='lwr_forward_kinematics',
        namespace='',
        name='lwr_forward_kinematics',
        parameters=[{
            "tool": LaunchConfiguration('tool'),
        }]
        )

    lwr_motion_node = Node(
        package='lwr_motion_program',
        executable='ptp_demo',
        namespace='',
        name='ptp_demo',
        parameters=[{
            "dt": LaunchConfiguration('dt'),
        }]
        )

    return LaunchDescription([
        dt_launch_arg,
        tool_launch_arg,
        joint_trajectory_node,
        lwr_forward_kinematics_node,
        lwr_motion_node
        ])

# https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html
