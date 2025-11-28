from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """
    Simplified MoveIt launch - just for visualization and basic planning.
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

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

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

    # Joint state publisher (for simulation/testing only - remove when using real robot)
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # RViz with MoveIt plugin
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
        ],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
