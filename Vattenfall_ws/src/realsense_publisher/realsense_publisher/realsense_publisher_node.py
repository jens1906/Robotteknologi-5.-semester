"""
Test Image Publisher - Publishes test images to simulate RealSense camera.

Publishes to the same topics as the real camera wrapper for testing:
- /camera/color/image_raw - Color image (640x480 RGB8 @ 30fps)
- /camera/aligned_depth_to_color/image_raw - Depth image (640x480 MONO16 @ 30fps)

Resolution: 640x480 @ 30fps
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
import cv2 as cv
import os


class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        
        # Declare parameters
        self.declare_parameter('test_image_path', 'src/realsense_publisher/image/realsense_capture_20251118_115022_color.png')
        self.declare_parameter('test_depth_path', 'src/realsense_publisher/image/realsense_capture_20251118_115022_depth.npy')
        
        # QoS profile matching realsense wrapper
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers - same topics as real wrapper
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', image_qos)
        self.depth_pub = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', image_qos)
        
        # Load test images
        test_image_path = self.get_parameter('test_image_path').get_parameter_value().string_value
        test_depth_path = self.get_parameter('test_depth_path').get_parameter_value().string_value
        
        if os.path.exists(test_image_path):
            self.test_image = cv.imread(test_image_path)
            # Convert BGR to RGB for publishing (wrapper publishes RGB8)
            self.test_image = cv.cvtColor(self.test_image, cv.COLOR_BGR2RGB)
        else:
            self.get_logger().warn(f'Test image not found: {test_image_path}, using blank image')
            self.test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        if os.path.exists(test_depth_path):
            self.test_depth = np.load(test_depth_path)
        else:
            self.get_logger().warn(f'Test depth not found: {test_depth_path}, using blank depth')
            self.test_depth = np.zeros((480, 640), dtype=np.uint16)
            
        
        # Timer for 30 fps
        self.timer = self.create_timer(1.0 / 30.0, self.publish_test_frames)
        self.get_logger().info('Test image publisher started - publishing @ 30fps')
    
    def numpy_to_image_msg(self, img, encoding):
        """Convert numpy array to ROS Image message."""
        msg = Image()
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        if len(img.shape) == 3:
            msg.step = img.shape[1] * img.shape[2] * img.itemsize
        else:
            msg.step = img.shape[1] * img.itemsize
        msg.data = img.tobytes()
        return msg

    def publish_test_frames(self):
        """Publish test images (matching real camera format)."""
        # Publish as RGB8 (same as wrapper) and MONO16
        color_msg = self.numpy_to_image_msg(self.test_image, 'rgb8')
        depth_msg = self.numpy_to_image_msg(self.test_depth, 'mono16')
        
        # Synchronized timestamps
        timestamp = self.get_clock().now().to_msg()
        color_msg.header.stamp = timestamp
        depth_msg.header.stamp = timestamp
        color_msg.header.frame_id = 'camera_color_optical_frame'
        depth_msg.header.frame_id = 'camera_depth_optical_frame'

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

