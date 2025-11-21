#!/usr/bin/env python3
"""
Launch file for UR3e workstation with MoveIt motion planning in RViz.
This starts the complete setup with visualization and motion planning capabilities.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    """Load a yaml file from package"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):
    """Launch setup function"""
    
    # Get workstation URDF
    workstation_xacro = PathJoinSubstitution([
        FindPackageShare("ur3e_workstation"),
        "urdf",
        "workstation.xacro"
    ])
    
    robot_description = Command([
        "xacro ", workstation_xacro,
        " use_mock_hardware:=true"
    ])
    
    # Get SRDF
    srdf_file = PathJoinSubstitution([
        FindPackageShare("ur3e_moveit_config"),
        "config",
        "ur3e.srdf"
    ])
    
    robot_description_semantic = Command(["cat ", srdf_file])
    
    # Kinematics config
    kinematics_yaml = load_yaml("ur3e_moveit_config", "config/kinematics.yaml")
    
    # Planning config
    ompl_planning_yaml = load_yaml("ur3e_moveit_config", "config/ompl_planning.yaml")
    
    # Joint limits
    joint_limits_yaml = load_yaml("ur3e_moveit_config", "config/joint_limits.yaml")
    
    # Controllers
    moveit_controllers = load_yaml("ur3e_moveit_config", "config/moveit_controllers.yaml")
    
    # Move group parameters
    move_group_params = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": kinematics_yaml,
        "robot_description_planning": joint_limits_yaml,
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": "default_planning_request_adapters/AddTimeOptimalParameterization default_planning_request_adapters/ResolveConstraintFrames default_planning_request_adapters/FixWorkspaceBounds default_planning_request_adapters/FixStartStateBounds default_planning_request_adapters/FixStartStateCollision default_planning_request_adapters/FixStartStatePathConstraints",
        "response_adapters": "default_planning_response_adapters/AddTimeOptimalParameterization default_planning_response_adapters/ValidateSolution default_planning_response_adapters/DisplayMotionPath",
        "planning_pipelines": ["ompl"],
        "ompl": ompl_planning_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": moveit_controllers.get("moveit_simple_controller_manager", {}),
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": 15.0,
        }],
    )
    
    # Joint State Publisher (for simulation)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{
            "source_list": ["move_group/fake_controller_joint_states"],
        }],
    )
    
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
    )
    
    # RViz with MoveIt config
    rviz_config = PathJoinSubstitution([
        FindPackageShare("ur3e_moveit_config"),
        "rviz",
        "moveit.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            move_group_params,
        ],
    )
    
    return [
        robot_state_publisher,
        joint_state_publisher,
        move_group_node,
        rviz_node,
    ]


def generate_launch_description():
    """Generate launch description"""
    
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
