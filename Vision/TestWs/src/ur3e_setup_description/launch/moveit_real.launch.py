from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml
import os
from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    """Load file content"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    """Load yaml file"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return {}

def generate_launch_description():
    """
    Launch MoveIt with real UR3e robot driver.
    - Subscribes to real robot joint_states (no conflicting publishers)
    - Allows planning with interactive markers
    - Shows both real robot state and planned trajectory
    """
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            description="Type/series of used UR robot.",
        )
    )

    ur_type = LaunchConfiguration("ur_type")
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur3e_setup_description"), "urdf", "ur3e_setup.urdf.xacro"]
            ),
            " ",
            "ur_type:=",
            ur_type,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher - subscribes to /joint_states from real robot
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Using real robot joint states only
    # No joint_state_publisher_gui to avoid conflicts

    # Get SRDF
    robot_description_semantic_content = Command(
        [
            "cat ",
            PathJoinSubstitution(
                [FindPackageShare("ur3e_setup_description"), "config", "ur3e_setup.srdf"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # Load YAML configs
    kinematics_yaml = load_yaml("ur3e_setup_description", "config/kinematics_moveit.yaml")
    ompl_planning_yaml = load_yaml("ur3e_setup_description", "config/ompl_moveit.yaml")
    controllers_yaml = load_yaml("ur3e_setup_description", "config/moveit_controllers.yaml")
    joint_limits_yaml = load_yaml("ur3e_setup_description", "config/joint_limits.yaml")
    
    # DEBUG: Print loaded configs
    print("DEBUG: Controllers YAML loaded:", controllers_yaml.get("/**", {}).get("ros__parameters", {}).keys() if controllers_yaml else "None")
    
    # Kinematics configuration - needs to be at robot_description_kinematics key
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    
    # MoveIt planning scene monitor settings
    planning_scene_monitor_params = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "joint_state_topic": "/joint_states",
        "wait_for_initial_state_timeout": 30.0,
    }

    # Planning pipeline configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            # Request adapters run BEFORE planning
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            # Response adapters run AFTER planning - this is where time parameterization happens!  
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
        }
    }
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Extract controller parameters properly
    controller_params = {}
    if controllers_yaml and "/**" in controllers_yaml and "ros__parameters" in controllers_yaml["/**"]:
        controller_params = controllers_yaml["/**"]["ros__parameters"]
    
    # Extract joint limits parameters for time parameterization
    # MoveIt expects joint limits under robot_description_planning key
    robot_description_planning = {}
    if joint_limits_yaml and "/**" in joint_limits_yaml and "ros__parameters" in joint_limits_yaml["/**"]:
        robot_description_planning = {"robot_description_planning": joint_limits_yaml["/**"]["ros__parameters"]}

    # MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,  # Joint limits wrapped under robot_description_planning
            ompl_planning_pipeline_config,
            planning_scene_monitor_params,
            controller_params,  # Pass extracted parameters directly
        ],
    )

    # RViz with MoveIt MotionPlanning plugin
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur3e_setup_description"), "rviz", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
