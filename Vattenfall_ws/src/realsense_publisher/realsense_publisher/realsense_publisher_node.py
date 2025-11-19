
Test = True
if Test == False:
    import pyrealsense2 as rs
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
import cv2 as cv

printlogger = True
test_image = cv.imread('src/realsense_publisher/image/realsense_capture_20251118_115022_color.png')
test_depth = np.load('src/realsense_publisher/image/realsense_capture_20251118_115022_depth.npy')


class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        
        # QoS profile for image topics (best effort for network transmission)
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.color_pub = self.create_publisher(Image, '/realsense/camera_color_pub', image_qos)
        self.depth_pub = self.create_publisher(Image, '/realsense/camera_depth_pub', image_qos)

        if not Test:
            # Real camera handles 30 fps natively
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(config)
            # Use callback immediately when camera data is ready
            self.timer = self.create_timer(0.001, self.publish_frames)
        else:
            # Test mode: limit to 30 fps (timer at 30 Hz)
            self.timer = self.create_timer(0.033, self.publish_frames)  # 1/30 = 0.0333 seconds
    
    def numpy_to_image_msg(self, img, encoding):
        msg = Image()
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = img.shape[1] * img.itemsize * (3 if len(img.shape) == 3 else 1)
        msg.data = img.tobytes()
        return msg


    def publish_frames(self):
        # Take frames from RealSense or test images and sync timestamps and send
        if not Test:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if color_frame and depth_frame:
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
        else:
            color_image = test_image
            depth_image = test_depth
        
        color_msg = self.numpy_to_image_msg(color_image, 'bgr8')
        depth_msg = self.numpy_to_image_msg(depth_image, 'mono16')
        
        timestamp = self.get_clock().now().to_msg()
        color_msg.header.stamp = timestamp
        depth_msg.header.stamp = timestamp
        color_msg.header.frame_id = 'realsense_camera_link'
        depth_msg.header.frame_id = 'realsense_depth_link'

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)

            

def main():
    rclpy.init()                    
    node = RealSensePublisher()     
    try:
        rclpy.spin(node)            
    finally:                        
        if not Test and hasattr(node, 'pipeline'):
            node.pipeline.stop()    
        rclpy.shutdown()            

if __name__ == '__main__':
    main()
