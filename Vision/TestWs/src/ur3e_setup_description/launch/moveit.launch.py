from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml
import os
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def launch_setup(context, *args, **kwargs):
    """
    Launch MoveIt with the UR3e setup.
    """
    ur_type = LaunchConfiguration("ur_type").perform(context)
    
    # Get package path
    pkg_path = get_package_share_directory("ur3e_setup_description")
    
    # Get URDF via xacro
    import subprocess
    xacro_file = os.path.join(pkg_path, "urdf", "ur3e_setup.urdf.xacro")
    robot_description_content = subprocess.check_output(
        ["xacro", xacro_file, f"ur_type:={ur_type}"]
    ).decode("utf-8")
    
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher (subscribes to joint_states from real robot)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Get SRDF
    srdf_file = os.path.join(pkg_path, "config", "ur3e_setup.srdf")
    with open(srdf_file, 'r') as file:
        robot_description_semantic_content = file.read()
    
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Load yaml files as dictionaries
    kinematics_config = load_yaml("ur3e_setup_description", "config/kinematics.yaml")
    ompl_config = load_yaml("ur3e_setup_description", "config/ompl_planning.yaml")
    
    # Extract parameters from ROS 2 parameter format
    kinematics_params = kinematics_config.get("/**", {}).get("ros__parameters", {})
    ompl_params = ompl_config.get("/**", {}).get("ros__parameters", {})

    # Create move_group parameters
    move_group_params = [
        robot_description,
        robot_description_semantic,
        kinematics_params,
        ompl_params,
        {
            "planning_pipelines": ["ompl"],
            "use_sim_time": False,
        }
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # RViz with MoveIt plugin
    rviz_config_file = os.path.join(pkg_path, "rviz", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_params,
        ],
    )

    # Joint state publisher GUI for manual control (comment out when using real robot)
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_gui,
        move_group_node,
        rviz_node,
    ]

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
