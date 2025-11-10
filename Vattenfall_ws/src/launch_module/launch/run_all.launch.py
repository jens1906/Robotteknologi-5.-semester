from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_publisher',
            executable='realsense_publisher_node',
            name='realsense_publisher',
            output='screen',
            parameters=[{'some_param': True}],
        ),
        Node(
            package='path_planning',
            executable='path_planning_node',
            name='path_planning',
            output='screen',
        ),
        Node(
            package='corrosion_detection',
            executable='corrosion_detection_node',
            name='corrosion_detection',
            output='screen',
        ),
        Node(
            package='parameterization',
            executable='parameterization_node',
            name='parameterization',
            output='screen',
        ),
        Node(
            package='tool_orientation',
            executable='tool_orientation_node',
            name='tool_orientation',
            output='screen',
            parameters=[{'dt': 0.1,'neighbor_range': 3}],
        ),
        #Node(
        #    package='user_interface',
        #    executable='user_interface_node',
        #    name='user_interface',
        #    output='screen',
        #    parameters=[{'dt': 0.1,'neighbor_range': 3}],
        #),
    ])
