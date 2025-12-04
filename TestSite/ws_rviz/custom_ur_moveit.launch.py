import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    
    # Arguments for workstation xacro
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    base_at_world_origin = LaunchConfiguration("base_at_world_origin")

    # Build MoveIt config
    # We use the standard ur_moveit_config for SRDF and other settings
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .to_moveit_configs()
    )

    # OVERRIDE robot_description with our workstation xacro
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
        " base_at_world_origin:=", base_at_world_origin,
    ])
    
    moveit_config.robot_description = {"robot_description": robot_description_content}

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    # Servo node (optional)
    # We need to load the servo yaml manually since we are not inside the package
    # But we can use FindPackageShare
    try:
        servo_yaml_path = PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "config", "ur_servo.yaml"])
        # We can't easily load yaml here inside OpaqueFunction without resolving path?
        # Actually MoveItConfigsBuilder handles it usually.
        # Let's skip servo for now or try to load it if needed.
        # The original script used load_yaml with get_package_share_directory.
        # We can do that too.
        ur_moveit_config_share = get_package_share_directory("ur_moveit_config")
        import yaml
        with open(os.path.join(ur_moveit_config_share, "config/ur_servo.yaml")) as f:
            servo_yaml = yaml.safe_load(f)
        servo_params = {"moveit_servo": servo_yaml}
        
        servo_node = Node(
            package="moveit_servo",
            condition=IfCondition(launch_servo),
            executable="servo_node",
            parameters=[
                moveit_config.to_dict(),
                servo_params,
            ],
            output="screen",
        )
    except Exception as e:
        print(f"Warning: Could not configure servo node: {e}")
        servo_node = None


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    nodes_to_start = [wait_robot_description, move_group_node, rviz_node]
    if servo_node:
        nodes_to_start.append(servo_node)

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    
    # Standard UR MoveIt args
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"))
    declared_arguments.append(DeclareLaunchArgument("ur_type", default_value="ur3e", description="Type/series of used UR robot."))
    declared_arguments.append(DeclareLaunchArgument("warehouse_sqlite_path", default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"), description="Path where the warehouse database should be stored"))
    declared_arguments.append(DeclareLaunchArgument("launch_servo", default_value="false", description="Launch Servo?"))
    declared_arguments.append(DeclareLaunchArgument("use_sim_time", default_value="false", description="Using or not time from simulation"))
    declared_arguments.append(DeclareLaunchArgument("publish_robot_description_semantic", default_value="true", description="MoveGroup publishes robot description semantic"))

    # Workstation args
    declared_arguments.append(DeclareLaunchArgument("robot_ip", default_value="192.168.56.101", description="IP address by which the robot can be reached."))
    declared_arguments.append(DeclareLaunchArgument("use_mock_hardware", default_value="true", description="Start robot with mock hardware mirroring command to its states."))
    declared_arguments.append(DeclareLaunchArgument("base_at_world_origin", default_value="false", description="Place robot base at the world origin (true/false)"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
