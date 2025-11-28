#!/usr/bin/env python3
"""
Line Visualizer - Converts Float64MultiArray line data to RViz markers
Subscribes to /tool_orientation/xyz_rotation and publishes visualization markers
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
import numpy as np


class LineVisualizer(Node):
    def __init__(self):
        super().__init__('line_visualizer')
        
        # Subscriber to the line topic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/tool_orientation/xyz_rotation',
            self.line_callback,
            10
        )
        
        # Publishers for visualization
        self.marker_array_pub = self.create_publisher(
            MarkerArray,
            '/line_visualization',
            10
        )
        
        self.line_pub = self.create_publisher(
            Marker,
            '/line_path',
            10
        )
        
        self.get_logger().info('Line Visualizer started')
        self.get_logger().info('Subscribing to: /tool_orientation/xyz_rotation')
        self.get_logger().info('Publishing to: /line_visualization (MarkerArray)')
        self.get_logger().info('Publishing to: /line_path (LINE_STRIP)')
    
    def line_callback(self, msg):
        """Process incoming line data and create visualization markers"""
        
        import numpy as np
        from scipy.spatial.transform import Rotation as Rot
        
        # Parse the data: [x1,y1,z1,qx1,qy1,qz1,qw1, x2,y2,z2,qx2,qy2,qz2,qw2, ...]
        data = msg.data
        
        if len(data) % 7 != 0:
            self.get_logger().error(f'Invalid data length: {len(data)} (must be multiple of 7)')
            return
        
        num_points = len(data) // 7
        self.get_logger().info(f'Received {num_points} points', throttle_duration_sec=2.0)
        
        # Transformation from mount frame (incoming data) to base_link frame
        # Robot base_link is at [0, -0.218, 0] relative to mount with rotation rpy=[-90°, 0, 180°]
        # Adjust X and Z if needed to fine-tune the transformation
        mount_to_base_translation = np.array([0.20, -0.218, 0.25])  # TODO: Adjust X and Z if needed
        mount_to_base_rotation = Rot.from_euler('xyz', [90, 0, 0], degrees=True)
        
        # Clear previous markers
        marker_array = MarkerArray()
        
        # Create coordinate frame markers for each point
        for i in range(num_points):
            idx = i * 7
            # Position in mount frame
            pos_mount = np.array([data[idx], data[idx+1], data[idx+2]])
            # Orientation in mount frame
            quat_mount = np.array([data[idx+3], data[idx+4], data[idx+5], data[idx+6]])
            
            # Transform position from mount to base_link
            pos_base = mount_to_base_rotation.apply(pos_mount) + mount_to_base_translation
            x, y, z = pos_base[0], pos_base[1], pos_base[2]
            
            # Transform orientation from mount to base_link
            rot_mount = Rot.from_quat(quat_mount)
            rot_base = mount_to_base_rotation * rot_mount
            quat_base = rot_base.as_quat()
            qx, qy, qz, qw = quat_base[0], quat_base[1], quat_base[2], quat_base[3]
            
            # Create axes markers (X, Y, Z)
            for axis_idx, (axis_name, color) in enumerate([
                ('x', [1.0, 0.0, 0.0]),  # Red
                ('y', [0.0, 1.0, 0.0]),  # Green
                ('z', [0.0, 0.0, 1.0])   # Blue
            ]):
                marker = Marker()
                marker.header.frame_id = 'base_link'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f'point_{i}'
                marker.id = i * 3 + axis_idx
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                
                # Position
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                
                # Orientation
                marker.pose.orientation.x = qx
                marker.pose.orientation.y = qy
                marker.pose.orientation.z = qz
                marker.pose.orientation.w = qw
                
                # Scale (arrow dimensions)
                marker.scale.x = 0.05  # Shaft diameter
                marker.scale.y = 0.01  # Arrow head diameter
                marker.scale.z = 0.01  # Arrow head length
                
                # Color
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 0.8
                
                # Rotate to show correct axis
                # The default arrow points along X-axis
                # For Y-axis, rotate 90° around Z
                # For Z-axis, rotate -90° around Y
                if axis_name == 'y':
                    # Rotate around Z by 90 degrees
                    import numpy as np
                    from scipy.spatial.transform import Rotation as Rot
                    q_base = Rot.from_quat([qx, qy, qz, qw])
                    q_rot = Rot.from_euler('z', 90, degrees=True)
                    q_combined = q_base * q_rot
                    q_final = q_combined.as_quat()
                    marker.pose.orientation.x = q_final[0]
                    marker.pose.orientation.y = q_final[1]
                    marker.pose.orientation.z = q_final[2]
                    marker.pose.orientation.w = q_final[3]
                elif axis_name == 'z':
                    # Rotate around Y by -90 degrees
                    from scipy.spatial.transform import Rotation as Rot
                    q_base = Rot.from_quat([qx, qy, qz, qw])
                    q_rot = Rot.from_euler('y', -90, degrees=True)
                    q_combined = q_base * q_rot
                    q_final = q_combined.as_quat()
                    marker.pose.orientation.x = q_final[0]
                    marker.pose.orientation.y = q_final[1]
                    marker.pose.orientation.z = q_final[2]
                    marker.pose.orientation.w = q_final[3]
                
                marker_array.markers.append(marker)
            
            # Add sphere at each point
            sphere = Marker()
            sphere.header.frame_id = 'base_link'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = f'sphere_{i}'
            sphere.id = num_points * 3 + i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.02
            sphere.scale.y = 0.02
            sphere.scale.z = 0.02
            sphere.color.r = 1.0
            sphere.color.g = 1.0
            sphere.color.b = 0.0
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)
        
        # Publish marker array
        self.marker_array_pub.publish(marker_array)
        
        # Create LINE_STRIP marker connecting all points
        line_marker = Marker()
        line_marker.header.frame_id = 'base_link'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'line_path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.005  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0
        
        # Add all transformed points to the line
        for i in range(num_points):
            idx = i * 7
            # Position in mount frame
            pos_mount = np.array([data[idx], data[idx+1], data[idx+2]])
            # Transform position from mount to base_link
            pos_base = mount_to_base_rotation.apply(pos_mount) + mount_to_base_translation
            
            point = Point()
            point.x = float(pos_base[0])
            point.y = float(pos_base[1])
            point.z = float(pos_base[2])
            line_marker.points.append(point)
        
        # Publish line
        self.line_pub.publish(line_marker)


def main(args=None):
    rclpy.init(args=args)
    node = LineVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
