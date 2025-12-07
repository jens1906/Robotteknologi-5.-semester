from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for the ArUco MoveIt client.
    
    This node subscribes to /aruco_pose and moves the UR3e robot to a position
    above the detected marker using MoveIt's Cartesian path planning.
    """
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base frame of the robot'
        ),
        
        DeclareLaunchArgument(
            'ee_frame',
            default_value='tool0',
            description='End-effector frame'
        ),
        
        DeclareLaunchArgument(
            'lift',
            default_value='0.20',
            description='Height above marker in meters (default: 0.20m)'
        ),
        
        DeclareLaunchArgument(
            'execute_motion',
            default_value='true',
            description='Whether to execute the planned motion'
        ),
        
        DeclareLaunchArgument(
            'velocity_scaling',
            default_value='0.1',
            description='Velocity scaling factor (0.0-1.0)'
        ),
        
        DeclareLaunchArgument(
            'acceleration_scaling',
            default_value='0.1',
            description='Acceleration scaling factor (0.0-1.0)'
        ),
        
        Node(
            package='aruco_moveit',
            executable='ur5_moveit_client',
            name='ur5_moveit_client',
            output='screen',
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'ee_frame': LaunchConfiguration('ee_frame'),
                'lift': LaunchConfiguration('lift'),
                'execute_motion': LaunchConfiguration('execute_motion'),
                'velocity_scaling': LaunchConfiguration('velocity_scaling'),
                'acceleration_scaling': LaunchConfiguration('acceleration_scaling'),
            }]
        ),
    ])
