"""
RealSense Camera Launch File

Launches the realsense2_camera node with proper configuration:
- Resolution: 640x480
- Framerate: 30 fps
- Depth aligned to color camera
- Synchronized color and depth frames

Topics published:
- /camera/color/image_raw - Color image (640x480 @ 30fps)
- /camera/aligned_depth_to_color/image_raw - Aligned depth (640x480 @ 30fps)
- /camera/color/camera_info - Color camera calibration
- /camera/aligned_depth_to_color/camera_info - Depth camera calibration

Usage:
    ros2 launch realsense_wrapper rs_launch.py
"""

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
                
                # Resolution and framerate - THIS IS KEY!
                'rgb_camera.profile': '640x480x30',
                'depth_module.profile': '848x480x30',  # D435 native depth resolution
                
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
