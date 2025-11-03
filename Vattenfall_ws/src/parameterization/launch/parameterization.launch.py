"""Launch file for parameterization node"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with parameterization node"""
    
    parameterization_node = Node(
        package='parameterization',
        executable='parameterization_node',
        name='parameterization_node',
        output='screen',
        parameters=[{
            'interpolation_method': 'rbf',
            'neighbors': 50,
            'normalize': False,
            'quality_sample_size': 1000,
            'status_publish_rate': 1.0,
        }]
    )
    
    return LaunchDescription([
        parameterization_node
    ])
