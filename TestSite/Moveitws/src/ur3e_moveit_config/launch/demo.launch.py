#!/usr/bin/env python3
"""
Standalone UR3e MoveIt demo launch file
This creates a complete simulation without needing ur_robot_driver
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
import os


def load_yaml(package_name, file_path):
    """Load a yaml file"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    
    # Get UR description package
    ur_type = 'ur3e'
    
    # Robot description from xacro
    xacro_file = os.path.join(
        get_package_share_directory('ur_description'),
        'urdf',
        ur_type + '.urdf.xacro'
    )
    
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            'name': ur_type,
            'ur_type': ur_type,
            'prefix': '""',
            'sim_gazebo': 'false',
            'sim_ignition': 'false',
            'simulation_controllers': '',
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # MoveIt configuration
    robot_description_semantic_file = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'srdf',
        ur_type + '.srdf.xacro'
    )
    
    robot_description_semantic_config = xacro.process_file(
        robot_description_semantic_file,
        mappings={'name': ur_type, 'prefix': ''}
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config.toxml()
    }
    
    # Kinematics config
    kinematics_yaml = load_yaml('ur_moveit_config', 'config/kinematics.yaml')
    
    # Planning pipeline config
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                               'default_planner_request_adapters/ResolveConstraintFrames '
                               'default_planner_request_adapters/FixWorkspaceBounds '
                               'default_planner_request_adapters/FixStartStateBounds '
                               'default_planner_request_adapters/FixStartStateCollision '
                               'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    
    ompl_planning_yaml = load_yaml('ur_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)
    
    # Trajectory execution
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
        ],
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ur_moveit_config'),
        'rviz',
        'view_robot.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        move_group_node,
        rviz_node,
    ])
