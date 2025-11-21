#!/usr/bin/env python3
"""
Complete standalone launch file for workstation with MoveIt and fake execution.
This allows motion planning and visualization without real hardware.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    
    # Get URDF
    workstation_xacro = PathJoinSubstitution([
        FindPackageShare("ur3e_workstation"),
        "urdf",
        "workstation.xacro"
    ])
    
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            workstation_xacro,
            " ur_type:=", ur_type,
        ]),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    # Get SRDF (for UR MoveIt config)
    ur_srdf = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"),
        "srdf",
        "ur.srdf.xacro"
    ])
    
    robot_description_semantic_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            ur_srdf,
            " name:=ur"
        ]),
        value_type=str
    )
    
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    
    # Kinematics
    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"),
        "config",
        ur_type,
        "default_kinematics.yaml"
    ])
    
    # Joint limits
    joint_limits_yaml = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"),
        "config",
        ur_type,
        "joint_limits.yaml"
    ])
    
    # Planning pipelines
    ompl_yaml = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"),
        "config",
        "ompl_planning.yaml"
    ])
    
    pilz_yaml = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"),
        "config",
        "pilz_industrial_motion_planner_planning.yaml"
    ])
    
    # Trajectory execution and moveit controllers for FAKE execution
    moveit_controllers = {
        "moveit_controller_manager": "moveit_fake_controller_manager/MoveItFakeControllerManager",
        "moveit_fake_controller_manager": {
            "controller_names": ["fake_ur_controller"],
            "fake_ur_controller": {
                "type": "Fake",
                "joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint", 
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint"
                ]
            }
        }
    }
    
    # Planning scene monitor parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )
    
    # Joint state publisher for fake execution
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            robot_description,
            {"source_list": ["move_group/fake_controller_joint_states"]},
        ],
    )
    
    # MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_yaml,
            pilz_yaml,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
            {"planning_pipelines": ["pilz_industrial_motion_planner", "chomp", "ompl", "stomp"]},
        ],
    )
    
    # RViz
    rviz_config = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"),
        "rviz",
        "view_robot.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_yaml,
            pilz_yaml,
        ],
    )
    
    # Collision matrix publisher
    collision_matrix_publisher = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ur3e_workstation",
                executable="publish_collision_matrix.py",
                output="screen",
                name="collision_matrix_publisher"
            )
        ]
    )
    
    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        move_group_node,
        rviz_node,
        collision_matrix_publisher,
    ]
    
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            description="Type/series of used UR robot.",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
