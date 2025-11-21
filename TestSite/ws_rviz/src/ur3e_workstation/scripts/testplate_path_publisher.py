#!/usr/bin/env python3
"""
Test plate path publisher - generates a scanning path along the curved test plate.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R


class TestPlatePathPublisher(Node):
    def __init__(self):
        super().__init__('testplate_path_publisher')
        
        self.publisher = self.create_publisher(Float64MultiArray, '/scanning_path', 10)
        self.timer = self.create_timer(2.0, self.publish_path)
        
        self.get_logger().info('Test Plate Path Publisher initialized')
        self.get_logger().info('Publishing scanning path...')
        
        self.waypoints = self.generate_testplate_path()
        self.get_logger().info(f'Generated path with {len(self.waypoints) // 12} waypoints')
    
    def generate_testplate_path(self):
        waypoints = []
        num_points = 10
        
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # IMPORTANT: These coordinates are reachable by the robot
            x = -0.05 + 0.1 * t  # -0.05 to 0.05m
            y = -0.6              # 0.25m FORWARD (reachable!)
            z = 0.25 + 0.05 * np.sin(t * np.pi)  # 0.25 to 0.30m
            
            position = np.array([x, y, z])
            
            # Orientation: pointing down, with -90° yaw correction
            euler_angles = [180, 0, -90]  # pitch, roll, yaw
            
            rotation = R.from_euler('xyz', euler_angles, degrees=True)
            rot_matrix = rotation.as_matrix()
            
            waypoint = list(rot_matrix.flatten()) + list(position)
            waypoints.extend(waypoint)
            
            if i == 0:
                self.get_logger().info(f'Start: pos=[{x:.3f}, {y:.3f}, {z:.3f}]')
            elif i == num_points - 1:
                self.get_logger().info(f'End: pos=[{x:.3f}, {y:.3f}, {z:.3f}]')
        
        return waypoints
    
    def publish_path(self):
        msg = Float64MultiArray()
        msg.data = self.waypoints
        self.publisher.publish(msg)
        
        num_waypoints = len(self.waypoints) // 12
        self.get_logger().info(f'Published scanning path: {num_waypoints} waypoints', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = TestPlatePathPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
