#!/usr/bin/env python3
"""
Launch file that combines the custom workstation with ros2_control and MoveIt.
This provides full motion planning with collision detection for the test setup.
Uses mock hardware for offline simulation.
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
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    reverse_ip = LaunchConfiguration("reverse_ip")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    
    # Include the UR control launch file which sets up ros2_control with controllers
    # This uses our custom workstation description
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "use_mock_hardware": use_mock_hardware,
            "robot_ip": robot_ip,
            "reverse_ip": reverse_ip,
            "kinematics_params_file": kinematics_params_file,
            "launch_rviz": "false",  # We'll launch RViz separately with MoveIt config
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "activate_joint_controller": "true",
            "headless_mode": "false",
            "launch_dashboard_client": "true",
            # Add controller update rate for smooth execution
            "update_rate_config_file": PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "config",
                "ur3e_update_rate.yaml"
            ]),
            # Script files for UR communication
            "script_filename": PathJoinSubstitution([
                FindPackageShare("ur_client_library"),
                "resources",
                "external_control.urscript"
            ]),
            "output_recipe_filename": PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "resources",
                "rtde_output_recipe.txt"
            ]),
            "input_recipe_filename": PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "resources",
                "rtde_input_recipe.txt"
            ]),
            # Use our custom workstation description launch file
            "description_launchfile": PathJoinSubstitution([
                FindPackageShare("ur3e_workstation"),
                "launch",
                "workstation_description.launch.py"
            ]),
        }.items()
    )
    
    # Include the UR MoveIt launch file for motion planning
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "use_mock_hardware": use_mock_hardware,
            "launch_rviz": "true",
            # Use our custom workstation description launch file
            "description_launchfile": PathJoinSubstitution([
                FindPackageShare("ur3e_workstation"),
                "launch",
                "workstation_description.launch.py"
            ]),
        }.items()
    )
    
    # Publish collision matrix after a delay to allow MoveIt to start
    collision_matrix_publisher = TimerAction(
        period=5.0,  # Wait 5 seconds for MoveIt to start
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
        ur_control_launch,
        ur_moveit_launch,
        collision_matrix_publisher,
    ]

def generate_launch_description():
    # Declare arguments
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur3e",
        description="Type/series of used UR robot."
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.56.101",
        description="IP address by which the robot can be reached."
    )
    
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description="Start robot with mock hardware mirroring command to its states."
    )
    
    reverse_ip_arg = DeclareLaunchArgument(
        "reverse_ip",
        default_value="0.0.0.0",
        description="IP address of the computer running the driver (this PC). Use 0.0.0.0 for auto-detection."
    )
    
    kinematics_params_file_arg = DeclareLaunchArgument(
        "kinematics_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("ur_description"),
            "config",
            "ur3e",
            "default_kinematics.yaml"
        ]),
        description="Kinematics calibration file for the robot."
    )

    return LaunchDescription([
        ur_type_arg,
        robot_ip_arg,
        use_mock_hardware_arg,
        reverse_ip_arg,
        kinematics_params_file_arg,
        OpaqueFunction(function=launch_setup)
    ])
