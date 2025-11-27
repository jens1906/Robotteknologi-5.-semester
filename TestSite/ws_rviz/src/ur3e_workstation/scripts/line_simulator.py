#!/usr/bin/env python3
"""
Line Simulator - Publishes a configurable line in robot base frame
Publishes points with positions and orientations as a flat list:
[x1,y1,z1,qx1,qy1,qz1,qw1, x2,y2,z2,qx2,qy2,qz2,qw2, ...]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R


class LineSimulator(Node):
    def __init__(self):
        super().__init__('line_simulator')
        
        # Publisher for the tool orientation topic
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/tool_orientation/xyz_rotation',
            10
        )
        
        # ===== CONFIGURABLE PARAMETERS =====
        # Line reference point (center of line) in robot base frame
        # UR3e workspace: X,Y: ±0.5m safe zone, Z: 0.15-0.6m
        # These coordinates are relative to base_link (robot base)
        self.line_center = np.array([0.0, -0.3, 0.2])  # [x, y, z] in meters - safe workspace position
        
        # Line orientation (Euler angles in degrees: roll, pitch, yaw)
        # This rotates the line direction in the base frame
        self.line_orientation_deg = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw] - horizontal line
        
        # Line length and direction
        self.line_length = 0.3  # Total length in meters
        self.line_direction = np.array([1.0, 0.0, 0.0])  # Direction vector (will be normalized)
        
        # Tool orientation at each point (Euler angles in degrees)
        # [0,180,0] rotates tool 180° around Y-axis so camera points forward instead of down
        self.tool_orientation_deg = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        
        # Number of points along the line
        self.num_points = 2
        # ===================================
        
        # Generate the line
        self.line_points = self.generate_line()
        
        # Timer to publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_line)
        
        self.get_logger().info('Line Simulator started')
        self.get_logger().info(f'Publishing {self.num_points} points on /tool_orientation/xyz_rotation')
        self.get_logger().info(f'Line center: {self.line_center}')
        self.get_logger().info(f'Line length: {self.line_length}m')
    
    def generate_line(self):
        """Generate line points with positions and orientations (in mount frame)"""
        
        # Normalize line direction
        direction = self.line_direction / np.linalg.norm(self.line_direction)
        
        # Create rotation matrix for line orientation
        line_rotation = R.from_euler('xyz', self.line_orientation_deg, degrees=True)
        
        # Rotate the direction vector
        rotated_direction = line_rotation.apply(direction)
        
        # Generate points along the line
        points = []
        for i in range(self.num_points):
            # Parameter along line: 0 at start, 1 at end
            t = i / max(1, self.num_points - 1)
            
            # Position: center + offset along rotated direction
            offset = (t - 0.5) * self.line_length  # Center the line
            position = self.line_center + rotated_direction * offset
            
            # Tool orientation (same for all points, but could be varied)
            tool_rot = R.from_euler('xyz', self.tool_orientation_deg, degrees=True)
            quat = tool_rot.as_quat()  # [qx, qy, qz, qw]
            
            # Append [x, y, z, qx, qy, qz, qw] in mount frame
            points.extend([
                position[0], position[1], position[2],
                quat[0], quat[1], quat[2], quat[3]
            ])
        
        return points
    
    def publish_line(self):
        """Publish the line points"""
        msg = Float64MultiArray()
        msg.data = self.line_points
        self.publisher.publish(msg)
        
        # Log first and last point for verification
        self.get_logger().info(
            f'Published line: '
            f'P1=[{self.line_points[0]:.3f}, {self.line_points[1]:.3f}, {self.line_points[2]:.3f}] '
            f'P{self.num_points}=[{self.line_points[-7]:.3f}, {self.line_points[-6]:.3f}, {self.line_points[-5]:.3f}]',
            throttle_duration_sec=5.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = LineSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
