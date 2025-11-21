#!/usr/bin/env python3
"""
Test path publisher for simulating scanning paths.
Publishes a simple test path to the /scanning_path topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R


class TestPathPublisher(Node):
    """Publishes a test path for visualization and execution testing."""
    
    def __init__(self):
        super().__init__('test_path_publisher')
        
        # Publisher
        self.publisher = self.create_publisher(Float64MultiArray, '/scanning_path', 10)
        
        self.get_logger().info('Test Path Publisher initialized')
        self.get_logger().info('Press Enter to publish a test path...')
    
    def create_simple_line_path(self, start_pos, end_pos, num_points=10):
        """Create a simple linear path from start to end position."""
        waypoints = []
        
        # Create interpolated positions
        for i in range(num_points):
            t = i / (num_points - 1)
            pos = start_pos + t * (end_pos - start_pos)
            
            # Create orientation pointing along negative Y-axis
            # Tool/end-effector pointing forward (negative Y direction)
            # X-axis points right, Z-axis points down, Y-axis points back
            # Rotate 90 degrees around X-axis to point Z-axis along -Y
            rot = R.from_euler('xyz', [90, 0, -90], degrees=True)
            rot_matrix = rot.as_matrix()
            
            # Flatten rotation matrix and append position
            waypoint = list(rot_matrix.flatten()) + list(pos)
            waypoints.extend(waypoint)
        
        return waypoints
    
    def create_curved_path(self, center, radius, height, num_points=20):
        """Create a curved scanning path (e.g., semicircle)."""
        waypoints = []
        
        for i in range(num_points):
            # Angle from 0 to 180 degrees
            angle = np.pi * i / (num_points - 1)
            
            # Position on semicircle
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2] + height
            pos = np.array([x, y, z])
            
            # Orientation: point camera toward center, angled down
            # Calculate angle to point at center
            direction = center - pos
            direction = direction / np.linalg.norm(direction)
            
            # Create rotation matrix to align Z-axis with direction
            z_axis = direction
            x_axis = np.array([1, 0, 0])
            y_axis = np.cross(z_axis, x_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, z_axis)
            
            rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
            
            # Flatten rotation matrix and append position
            waypoint = list(rot_matrix.flatten()) + list(pos)
            waypoints.extend(waypoint)
        
        return waypoints
    
    def create_grid_scan_path(self, start_pos, width, height, rows=5, cols=5):
        """Create a grid scanning pattern."""
        waypoints = []
        
        # Simple downward-pointing orientation
        rot = R.from_euler('xyz', [180, 0, 0], degrees=True)
        rot_matrix = rot.as_matrix()
        rot_flat = list(rot_matrix.flatten())
        
        # Create grid points
        for row in range(rows):
            for col in range(cols):
                # Alternate direction for efficiency (lawnmower pattern)
                if row % 2 == 0:
                    x = start_pos[0] + (width * col / (cols - 1))
                else:
                    x = start_pos[0] + (width * (cols - 1 - col) / (cols - 1))
                
                y = start_pos[1] + (height * row / (rows - 1))
                z = start_pos[2]
                
                waypoint = rot_flat + [x, y, z]
                waypoints.extend(waypoint)
        
        return waypoints
    
    def publish_path(self, waypoints):
        """Publish the path to the topic."""
        msg = Float64MultiArray()
        msg.data = waypoints
        
        self.publisher.publish(msg)
        
        num_waypoints = len(waypoints) // 12
        self.get_logger().info(f'Published path with {num_waypoints} waypoints')
        self.get_logger().info(f'Total data points: {len(waypoints)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TestPathPublisher()
    
    try:
        print("\nSelect path type:")
        print("1. Simple line path")
        print("2. Curved path (semicircle)")
        print("3. Grid scan pattern")
        print("4. Custom path")
        
        choice = input("\nEnter choice (1-4): ").strip()
        
        if choice == '1':
            # Simple line from one point to another
            # Original was: start = [0.3, 0.2, 0.3], end = [0.3, -0.2, 0.3]
            # Rotate -90° around Z (swap X,Y with Y -> -Y, X -> X becomes Y -> -X, X -> Y)
            # Then move down -0.5m on Z
            # Original line goes from (0.3, 0.2) to (0.3, -0.2) in XY plane
            # After -90° rotation: X'=-Y, Y'=X
            # (0.3, 0.2) -> (-0.2, 0.3)
            # (0.3, -0.2) -> (0.2, 0.3)
            # Z: 0.3 - 0.5 = -0.2
            start = np.array([-0.2, -0.3, 0])
            end = np.array([0.2, -0.3, 0])
            waypoints = node.create_simple_line_path(start, end, num_points=10)
            
        elif choice == '2':
            # Curved path
            center = np.array([0.4, 0.0, 0.0])
            waypoints = node.create_curved_path(center, radius=0.2, height=0.3, num_points=20)
            
        elif choice == '3':
            # Grid scan pattern
            start = np.array([0.2, -0.15, 0.3])
            waypoints = node.create_grid_scan_path(start, width=0.3, height=0.3, rows=5, cols=5)
            
        elif choice == '4':
            # Custom - user can modify this section
            print("\nEdit the script to add your custom path!")
            return
            
        else:
            print("Invalid choice!")
            return
        
        # Publish the path
        node.publish_path(waypoints)
        
        print("\nPath published! Check RViz for visualization.")
        print("To execute the path, call one of these services:")
        print("  ros2 service call /execute_cartesian_path std_srvs/srv/Trigger")
        print("  ros2 service call /execute_point_to_point std_srvs/srv/Trigger")
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
