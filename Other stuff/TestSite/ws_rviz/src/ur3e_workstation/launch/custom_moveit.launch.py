import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context)
    
    # 1. Build MoveIt Configuration
    # We use the standard 'ur_moveit_config' package as a base,
    # but we override the URDF and SRDF with our custom workstation files.
    
    workstation_pkg = FindPackageShare("ur3e_workstation").find("ur3e_workstation")
    xacro_path = os.path.join(workstation_pkg, "urdf", "workstation.xacro")
    srdf_path = os.path.join(workstation_pkg, "urdf", "workstation.srdf")
    
    # Mappings for the URDF xacro
    xacro_mappings = {
        "ur_type": ur_type,
        "use_mock_hardware": use_mock_hardware,
        "base_at_world_origin": "false"
    }

    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description(file_path=xacro_path, mappings=xacro_mappings)
        .robot_description_semantic(file_path=srdf_path)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Override OMPL config with our custom one
    ompl_config_path = os.path.join(workstation_pkg, "config", "ompl_planning.yaml")
    if os.path.exists(ompl_config_path):
        with open(ompl_config_path, "r") as f:
            custom_ompl = yaml.safe_load(f)
        if "ompl" in moveit_config.planning_pipelines:
            moveit_config.planning_pipelines["ompl"].update(custom_ompl)

    # 2. Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
            {"start_state_max_bounds_error": 0.1},
        ],
    )

    # 3. RViz
    # We try to find a local RViz config, otherwise use the default
    rviz_config = os.path.join(workstation_pkg, "rviz", "workstation.rviz")
    if not os.path.exists(rviz_config):
        # Fallback to generic UR config if local doesn't exist
        rviz_config = os.path.join(
            FindPackageShare("ur_moveit_config").find("ur_moveit_config"),
            "rviz",
            "view_robot.rviz"
        )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False},
        ],
    )
    
    return [move_group_node, rviz_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        OpaqueFunction(function=launch_setup)
    ])
