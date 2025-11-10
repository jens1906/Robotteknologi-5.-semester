#!/usr/bin/env python3
"""
Simplified launch file for UR3e using official ur_moveit_config package
This uses the accurate UR3e model from Universal Robots ROS2 packages
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Declare arguments
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur3e',
        description='Type of UR robot (ur3e, ur5e, ur10e, etc.)'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Start with fake hardware (simulation)'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # Include the official UR MoveIt config launch file
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items()
    )
    
    return LaunchDescription([
        ur_type_arg,
        use_fake_hardware_arg,
        launch_rviz_arg,
        ur_moveit_launch
    ])
