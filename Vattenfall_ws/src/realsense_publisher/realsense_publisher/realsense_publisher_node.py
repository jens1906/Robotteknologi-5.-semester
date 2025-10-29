import pyrealsense2 as rs
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        self.color_pub = self.create_publisher(Image, 'camera/color', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth', 10)
        
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.timer = self.create_timer(0.033, self.publish_frames)
    
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
        frames = self.pipeline.wait_for_frames()
        
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        if color_frame and depth_frame:
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            color_msg = self.numpy_to_image_msg(color_image, 'bgr8')
            depth_msg = self.numpy_to_image_msg(depth_image, 'mono16')
            
            timestamp = self.get_clock().now().to_msg()
            color_msg.header.stamp = timestamp
            depth_msg.header.stamp = timestamp
            color_msg.header.frame_id = 'camera_link'
            depth_msg.header.frame_id = 'camera_link'
            
            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)

def main():
    rclpy.init()
    node = RealSensePublisher()
    rclpy.spin(node)
    node.pipeline.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
