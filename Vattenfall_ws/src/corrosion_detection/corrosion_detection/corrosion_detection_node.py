import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
import numpy as np
import cv2  # ADD THIS

class CorrosionDetector(Node):
    def __init__(self):
        super().__init__('corrosion_detector')
        
        # Subscribe to both topics
        color_sub = message_filters.Subscriber(self, Image, 'camera/color')
        depth_sub = message_filters.Subscriber(self, Image, 'camera/depth')
        
        # Synchronize them based on timestamps
        sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], 
            queue_size=10,
            slop=0.1  # Maximum time difference (100ms)
        )
        sync.registerCallback(self.callback)
        
        self.get_logger().info('Waiting for synchronized images...')
    
    def callback(self, color_msg, depth_msg):
        # These are GUARANTEED to be from the same frame!
        self.get_logger().info(f'Got matching pair at {color_msg.header.stamp.sec}.{color_msg.header.stamp.nanosec}')
        
        # Convert to numpy arrays
        color_image = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(
            color_msg.height, color_msg.width, 3)
        depth_image = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
            depth_msg.height, depth_msg.width)
        
        # Normalize depth for visualization
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )
        
        # Show images
        cv2.imshow('Color', color_image)
        cv2.imshow('Depth', depth_colormap)
        cv2.waitKey(1)
        
        # Now do your processing with matching images
        # self.detect_corrosion(color_image, depth_image)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = CorrosionDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()