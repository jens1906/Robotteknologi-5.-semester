# RealSense Wrapper

Simple ROS2 package to launch the Intel RealSense camera with correct configuration.

## Installation

```bash
# Install RealSense ROS2 wrapper
sudo apt install ros-jazzy-realsense2-camera

# Build this package
cd ~/Vattenfall_ws
colcon build --packages-select realsense_wrapper
source install/setup.bash
```

## Usage

### Basic Launch (640x480 @ 30fps with alignment)

```bash
ros2 launch realsense_wrapper rs_launch.py
```

### With Point Cloud

```bash
ros2 launch realsense_wrapper rs_launch.py enable_pointcloud:=true
```

### With Depth Filters (cleaner depth data)

```bash
ros2 launch realsense_wrapper rs_launch.py enable_filters:=true
```

## Topics Published

### Images
- `/camera/color/image_raw` - Color image (640x480 BGR8 @ 30fps)
- `/camera/aligned_depth_to_color/image_raw` - Depth aligned to color (640x480 MONO16 @ 30fps)

### Camera Info (Calibration)
- `/camera/color/camera_info` - Color camera intrinsics/extrinsics
- `/camera/aligned_depth_to_color/camera_info` - Aligned depth camera info

### Optional (if enabled)
- `/camera/depth/color/points` - Point cloud (sensor_msgs/PointCloud2)

## Key Configuration

The launch file configures:

1. **Resolution:** 640x480 @ 30fps
   ```python
   'rgb_camera.profile': '640x480x30'
   'depth_module.profile': '640x480x30'
   ```

2. **Alignment:** Depth is aligned to color camera perspective
   ```python
   'align_depth.enable': True
   ```
   This means depth pixel (x,y) corresponds exactly to color pixel (x,y)!

3. **Synchronization:** Color and depth frames are synchronized
   ```python
   'enable_sync': True
   ```
   Both images have matching timestamps from the same capture moment.

## View Images

```bash
# View color
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw

# View aligned depth
ros2 run rqt_image_view rqt_image_view /camera/aligned_depth_to_color/image_raw
```

## Example Subscriber

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealsenseSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_sub')
        self.bridge = CvBridge()
        
        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw', 
            self.color_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, 10)
    
    def color_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # cv_image is 640x480 BGR8
    
    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        # depth_image is 640x480, values in millimeters
        # Aligned to color: depth_image[y,x] matches cv_image[y,x]
```

## Different Resolutions

Edit `launch/rs_launch.py` to change resolution:

```python
# 720p
'rgb_camera.profile': '1280x720x30',
'depth_module.profile': '1280x720x30',

# 60 fps
'rgb_camera.profile': '640x480x60',
'depth_module.profile': '640x480x60',
```

Available profiles depend on your camera model. Common options:
- 640x480 @ 6, 15, 30, 60 fps
- 1280x720 @ 6, 15, 30 fps
- 1920x1080 @ 6, 15, 30 fps (color only on some models)

## Troubleshooting

### Camera not detected
```bash
rs-enumerate-devices
```

### USB permissions
```bash
sudo usermod -a -G video $USER
# Then log out and back in
```

### No topics appearing
```bash
# Check if node is running
ros2 node list

# Check topics
ros2 topic list | grep camera

# Check if wrapper is installed
ros2 pkg list | grep realsense
```

Should see `realsense2_camera` in the list. If not:
```bash
sudo apt install ros-jazzy-realsense2-camera
```
