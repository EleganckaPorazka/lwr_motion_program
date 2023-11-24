import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    # time step dt
    dt_launch_arg = DeclareLaunchArgument(
        "dt", default_value=TextSubstitution(text="0.01")
    )
    #TODO: 'dt' parameter shall be set also for the inv_kin_launch_include, and along it, 'clik_gains'
    
    # end effector position (in meters) and orientation (in quaternions), relative to the last link's frame: [x,y,z,qx,qy,qz,qw]
    tool_launch_arg = DeclareLaunchArgument(
        "tool", default_value=TextSubstitution(text="[0.0,0.0,0.0,0.0,0.0,0.0,1.0]")
    )
    
    cartesian_trajectory_node = Node(
        package='trajectory_generator',
        executable='cartesian_sinusoidal_trajectory',
        namespace='',
        name='cartesian_sinusoidal_trajectory'
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
    
    inv_kin_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('inverse_kinematics'),
                'launch/inv_kin.launch.py'))
        )

    lwr_motion_node = Node(
        package='lwr_motion_program',
        executable='lin_demo',
        namespace='',
        name='lin_demo',
        parameters=[{
            "dt": LaunchConfiguration('dt'),
        }]
        )

    return LaunchDescription([
        dt_launch_arg,
        tool_launch_arg,
        cartesian_trajectory_node,
        lwr_forward_kinematics_node,
        inv_kin_launch_include,
        lwr_motion_node
        ])

# https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html
