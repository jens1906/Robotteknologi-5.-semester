#!/usr/bin/env python3
"""
Client-only launch file for MoveIt RViz interface.
This connects to an existing robot running on another PC and only launches:
- Robot description with test setup
- MoveIt planning interface 
- RViz for visualization and planning

Does NOT launch:
- Robot drivers
- Controllers
- Hardware interfaces
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get launch configurations
    ur_type = LaunchConfiguration("ur_type")
    
    # Include only the robot description (no controllers/drivers)
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur3e_workstation"),
                "launch",
                "workstation_description.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": ur_type,
        }.items()
    )
    
    # Include MoveIt planning interface with RViz
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "use_mock_hardware": "false",  # Connect to real robot topics
            "launch_rviz": "true",
            "launch_servo": "false",  # Don't launch servo since robot is external
            # Use our custom workstation description
            "description_launchfile": PathJoinSubstitution([
                FindPackageShare("ur3e_workstation"),
                "launch",
                "workstation_description.launch.py"
            ]),
        }.items()
    )
    
    # Publish collision matrix after delay to allow MoveIt to start
    collision_matrix_publisher = TimerAction(
        period=8.0,  # Wait 8 seconds for MoveIt to fully initialize
        actions=[
            Node(
                package="ur3e_workstation",
                executable="publish_collision_matrix.py",
                output="screen",
                name="collision_matrix_publisher"
            )
        ]
    )
    
    return [
        robot_description_launch,
        moveit_launch,
        collision_matrix_publisher,
    ]

def generate_launch_description():
    # Declare arguments
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur3e",
        description="Type/series of used UR robot."
    )

    return LaunchDescription([
        ur_type_arg,
        OpaqueFunction(function=launch_setup)
    ])