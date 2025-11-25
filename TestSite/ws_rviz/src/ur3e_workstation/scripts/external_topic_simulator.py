#!/usr/bin/env python3
"""
External topic simulator - publishes to /tool_orientation/xyz_rotation topic
to simulate what another PC would send. This is for testing the external line import.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R


class ExternalTopicSimulator(Node):
    def __init__(self):
        super().__init__('external_topic_simulator')
        
        # Publish to the external topic name
        self.publisher = self.create_publisher(Float64MultiArray, '/tool_orientation/xyz_rotation', 10)
        self.timer = self.create_timer(3.0, self.publish_path)
        
        self.get_logger().info('External Topic Simulator initialized')
        self.get_logger().info('Publishing to /tool_orientation/xyz_rotation topic...')
        self.get_logger().info('Format: [x,y,z, r11,r12,r13,r21,r22,r23,r31,r32,r33,...]')
        
        self.waypoints = self.generate_scanning_path()
        num_points = len(self.waypoints) // 12
        self.get_logger().info(f'Generated simulated path with {num_points} waypoints')
    
    def generate_scanning_path(self):
        """Generate a scanning path in the correct format: position first, then rotation matrix."""
        waypoints = []
        num_points = 15  # More points for a better visualization
        
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # Create a curved path in XYZ
            x = -0.1 + 0.2 * t  # -0.1 to 0.1m (left to right)
            y = -0.5 - 0.1 * np.sin(t * 2 * np.pi)  # Sine wave in Y
            z = 0.3 + 0.05 * np.cos(t * 3 * np.pi)  # Cosine wave in Z
            
            # Create varying tool orientations
            # Base orientation: pointing down with some rotation
            pitch = 170 + 20 * np.sin(t * np.pi)  # Vary pitch from 150° to 190°
            roll = 10 * np.sin(t * 2 * np.pi)     # Vary roll ±10°
            yaw = -90 + 30 * np.cos(t * np.pi)    # Vary yaw from -120° to -60°
            
            rotation = R.from_euler('xyz', [pitch, roll, yaw], degrees=True)
            rot_matrix = rotation.as_matrix()
            
            # Correct format: [x,y,z, r11,r12,r13,r21,r22,r23,r31,r32,r33]
            # Position FIRST, then rotation matrix
            waypoint = [x, y, z] + list(rot_matrix.flatten())
            waypoints.extend(waypoint)
            
            if i == 0:
                self.get_logger().info(f'Start: pos=[{x:.3f}, {y:.3f}, {z:.3f}] (position first format)')
            elif i == num_points - 1:
                self.get_logger().info(f'End: pos=[{x:.3f}, {y:.3f}, {z:.3f}] (position first format)')
        
        return waypoints
    
    def publish_path(self):
        """Publish the simulated path data."""
        msg = Float64MultiArray()
        msg.data = self.waypoints
        self.publisher.publish(msg)
        
        num_waypoints = len(self.waypoints) // 12
        self.get_logger().info(f'Published external path data: {num_waypoints} waypoints', throttle_duration_sec=10.0)
        self.get_logger().info(f'Total data elements: {len(self.waypoints)} (should be multiple of 12)', throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = ExternalTopicSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()