from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from pathlib import Path

def generate_launch_description():
    # Get path to realsense_wrapper launch file
    #realsense_wrapper_share = FindPackageShare('realsense_wrapper').find('realsense_wrapper')
    #realsense_launch = Path(realsense_wrapper_share) / 'launch' / 'rs_launch.py'
    
    return LaunchDescription([
        # Include RealSense wrapper launch
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(str(realsense_launch))
        #),
        Node(
            package='realsense_publisher',
            executable='realsense_publisher_node',
            name='realsense_publisher',
            output='screen',
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
            parameters=[{
                'dt': 0.1,
                'neighbor_range': 3,
                'use_identity_orientation': True,
                'orientation_rotation_axis': 'x',
                'orientation_rotation_angle_deg': 0
            }],
        ),
        #Node(
        #    package='user_interface',
        #    executable='user_interface_node',
        #    name='user_interface',
        #    output='screen',
        #    parameters=[{'dt': 0.1,'neighbor_range': 3}],
        #),
    ])
