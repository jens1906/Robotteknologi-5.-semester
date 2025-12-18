#!/usr/bin/env python3
"""
Launch file to publish the workstation robot description with ros2_control support.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    
    # Get workstation URDF with ros2_control
    workstation_xacro = PathJoinSubstitution([
        FindPackageShare("ur3e_workstation"),
        "urdf",
        "workstation.xacro"
    ])

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        workstation_xacro,
        " ur_type:=", ur_type,
        " robot_ip:=", robot_ip,
        " use_mock_hardware:=", use_mock_hardware,
    ])

    robot_description = {"robot_description": robot_description_content}

    # Publish the robot description using robot_state_publisher
    # The UR control launch expects this to be running
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    return [robot_state_publisher]


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            description="Type/series of used UR robot.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="IP address by which the robot can be reached.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
