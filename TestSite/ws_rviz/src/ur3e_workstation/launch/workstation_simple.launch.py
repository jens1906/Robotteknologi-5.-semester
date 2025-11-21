#!/usr/bin/env python3
"""
Simple workstation visualization launch file for UR3e lab setup.
Only starts robot_state_publisher and joint_state_publisher.
Launch RViz separately to avoid snap conflicts.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description="Start robot with mock hardware mirroring command to its states."
    )

    # Path to workstation xacro file
    workstation_xacro = PathJoinSubstitution([
        FindPackageShare("ur3e_workstation"),
        "urdf",
        "workstation.xacro"
    ])

    # Robot State Publisher - publishes the complete workstation model
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command([
                "xacro ", workstation_xacro,
                " use_mock_hardware:=", LaunchConfiguration("use_mock_hardware")
            ])
        }]
    )

    # Joint State Publisher GUI - allows manual joint control
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node
    ])
