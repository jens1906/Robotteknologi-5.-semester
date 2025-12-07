from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for customization
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='Camera unique name'
        ),
        
        DeclareLaunchArgument(
            'enable_pointcloud',
            default_value='false',
            description='Enable point cloud generation'
        ),
        
        DeclareLaunchArgument(
            'enable_filters',
            default_value='false',
            description='Enable depth filters for cleaner data'
        ),
        
        # RealSense camera node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=LaunchConfiguration('camera_name'),
            namespace='',
            parameters=[{
                # Camera naming
                'camera_name': LaunchConfiguration('camera_name'),
                
                # Enable streams
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_accel': False,
                'enable_gyro': False,
                
                # Resolution and framerate
                # Color at 1280x720, depth at 640x480 (D435 native), aligned depth will be 1280x720
                'rgb_camera.color_profile': '1280x720x15',
                'depth_module.depth_profile': '640x480x15',
                
                # Alignment - THIS ENSURES DEPTH ALIGNS TO COLOR!
                'align_depth.enable': True,
                
                # Synchronization - THIS ENSURES MATCHING TIMESTAMPS!
                'enable_sync': True,
                
                # Point cloud (optional)
                'pointcloud.enable': LaunchConfiguration('enable_pointcloud'),
                
                # Filters (optional, for cleaner depth)
                'decimation_filter.enable': LaunchConfiguration('enable_filters'),
                'spatial_filter.enable': LaunchConfiguration('enable_filters'),
                'temporal_filter.enable': LaunchConfiguration('enable_filters'),
                'hole_filling_filter.enable': LaunchConfiguration('enable_filters'),
                
                # Other settings
                'initial_reset': True,
                'unite_imu_method': 0,  # Fixed: use integer instead of string
            }],
            output='screen',
            emulate_tty=True,
        )
    ])
