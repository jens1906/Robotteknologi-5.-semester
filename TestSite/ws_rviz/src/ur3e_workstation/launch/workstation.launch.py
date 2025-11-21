#!/usr/bin/env python3
"""
Workstation visualization launch file for UR3e lab setup with MoveIt motion planning.
Loads complete workstation model (table, mount, UR3e, test plate, camera) in RViz with motion planning capabilities.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    
    use_moveit_arg = DeclareLaunchArgument(
        "use_moveit",
        default_value="false",
        description="Enable MoveIt motion planning capabilities."
    )

    # Path to workstation xacro file
    workstation_xacro = PathJoinSubstitution([
        FindPackageShare("ur3e_workstation"),
        "urdf",
        "workstation.xacro"
    ])

    # Robot description
    robot_description_content = Command([
        "xacro ", workstation_xacro,
        " use_mock_hardware:=", LaunchConfiguration("use_mock_hardware")
    ])
    
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher - publishes the complete workstation model
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    # SRDF file for MoveIt
    robot_description_semantic_content = Command([
        "cat ",
        PathJoinSubstitution([
            FindPackageShare("ur3e_workstation_moveit_config"),
            "config",
            "ur3e_workstation.srdf"
        ])
    ])
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # Kinematics config
    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare("ur3e_workstation_moveit_config"),
        "config",
        "kinematics.yaml"
    ])

    # Planning config
    ompl_planning_yaml = PathJoinSubstitution([
        FindPackageShare("ur3e_workstation_moveit_config"),
        "config",
        "ompl_planning.yaml"
    ])

    # MoveIt controllers
    moveit_controllers_yaml = PathJoinSubstitution([
        FindPackageShare("ur3e_workstation_moveit_config"),
        "config",
        "moveit_controllers.yaml"
    ])

    # MoveGroup node (only when use_moveit is true)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
            moveit_controllers_yaml,
            {
                "planning_pipelines": ["ompl"],
                "use_sim_time": False
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_moveit"))
    )

    # Joint State Publisher (for fake execution when using MoveIt)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            robot_description,
            {"source_list": ["move_group/fake_controller_joint_states"]},
        ],
        condition=IfCondition(LaunchConfiguration("use_moveit"))
    )

    # Joint State Publisher GUI - allows manual joint control (when NOT using MoveIt)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_moveit"))
    )

    # RViz - visualization (use MoveIt config if enabled, otherwise use simple config)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur3e_workstation_moveit_config"),
        "config",
        "moveit.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
        ],
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        use_moveit_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        move_group_node,
        rviz_node
    ])
