"""
Launch file for parameterization node.

Uses Amersdorfer et al. (2021) conformal parameterization approach
for equidistant path planning on curved surfaces.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description with conformal parameterization node"""
    
    # Declare launch arguments
    interpolation_method_arg = DeclareLaunchArgument(
        'interpolation_method',
        default_value='rbf',
        description='Interpolation method for (u,v) to (x,y,z) mapping'
    )
    
    neighbors_arg = DeclareLaunchArgument(
        'neighbors',
        default_value='50',
        description='Number of neighbors for local RBF interpolation'
    )
    
    metric_neighbors_arg = DeclareLaunchArgument(
        'metric_neighbors',
        default_value='20',
        description='Number of neighbors for metric tensor computation'
    )
    
    conformal_iterations_arg = DeclareLaunchArgument(
        'conformal_iterations',
        default_value='5',
        description='Number of conformal correction iterations'
    )
    
    conformal_alpha_arg = DeclareLaunchArgument(
        'conformal_alpha',
        default_value='0.5',
        description='Conformal correction step size (0 < alpha < 1)'
    )
    
    parameterization_node = Node(
        package='parameterization',
        executable='parameterization_node',
        name='parameterization_node',
        output='screen',
        parameters=[{
            'interpolation_method': LaunchConfiguration('interpolation_method'),
            'neighbors': LaunchConfiguration('neighbors'),
            'metric_neighbors': LaunchConfiguration('metric_neighbors'),
            'quality_sample_size': 1000,
            'status_publish_rate': 1.0,
            'conformal_iterations': LaunchConfiguration('conformal_iterations'),
            'conformal_alpha': LaunchConfiguration('conformal_alpha'),
        }]
    )
    
    return LaunchDescription([
        interpolation_method_arg,
        neighbors_arg,
        metric_neighbors_arg,
        conformal_iterations_arg,
        conformal_alpha_arg,
        parameterization_node
    ])
