"""
Example script to publish a point cloud to the parameterization node.

This script demonstrates how to:
1. Load a point cloud from a PLY file (or generate synthetic data)
2. Convert it to PointCloud2 message
3. Publish it to the parameterization node
4. Query interpolation services
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import time


def create_pointcloud2(points, frame_id='map'):
    """
    Create a PointCloud2 message from numpy array.
    
    Args:
        points: Nx3 numpy array of XYZ points
        frame_id: Frame ID for the point cloud
        
    Returns:
        PointCloud2 message
    """
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.header.frame_id = frame_id
    
    msg.height = 1
    msg.width = len(points)
    
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    
    buffer = []
    for point in points:
        buffer.append(struct.pack('fff', point[0], point[1], point[2]))
    
    msg.data = b''.join(buffer)
    
    return msg


def generate_synthetic_surface(n_points=1000):
    """
    Generate a synthetic surface for testing.
    Creates a wavy surface.
    
    Returns:
        Nx3 numpy array of points
    """
    x = np.linspace(-5, 5, int(np.sqrt(n_points)))
    y = np.linspace(-5, 5, int(np.sqrt(n_points)))
    X, Y = np.meshgrid(x, y)
    
    # Create wavy surface
    Z = 2 * np.sin(X) * np.cos(Y) + 5
    
    points = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
    
    return points


def load_ply_file(filepath):
    """
    Load point cloud from PLY file.
    
    Requires open3d:
        pip install open3d
    """
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(filepath)
        points = np.asarray(pcd.points)
        return points
    except ImportError:
        print("Open3D not installed. Install with: pip install open3d")
        return None


class PointCloudPublisher(Node):
    """Node to publish point cloud data"""
    
    def __init__(self, points):
        super().__init__('point_cloud_publisher')
        
        self.points = points
        
        self.publisher = self.create_publisher(
            PointCloud2,
            '/point_cloud',
            10
        )
        
        # Wait a bit for subscriber to connect
        time.sleep(1.0)
        
        # Publish the point cloud
        self.publish_point_cloud()
        
    def publish_point_cloud(self):
        """Publish the point cloud"""
        msg = create_pointcloud2(self.points, frame_id='map')
        self.publisher.publish(msg)
        self.get_logger().info(f'Published point cloud with {len(self.points)} points')


def main():
    """Main function"""
    rclpy.init()
    
    print("=" * 70)
    print("  Point Cloud Publisher Example")
    print("=" * 70)
    
    # Choose data source
    print("\nData source:")
    print("  1. Generate synthetic surface")
    print("  2. Load from PLY file")
    
    choice = input("\nChoose option (1/2) [default: 1]: ").strip()
    
    if choice == '2':
        filepath = input("Enter PLY file path: ").strip()
        points = load_ply_file(filepath)
        if points is None:
            print("Failed to load PLY file. Using synthetic data instead.")
            points = generate_synthetic_surface()
    else:
        n_points = input("Number of points [default: 1000]: ").strip()
        n_points = int(n_points) if n_points else 1000
        print(f"\nGenerating synthetic surface with {n_points} points...")
        points = generate_synthetic_surface(n_points)
    
    print(f"Loaded {len(points)} points")
    print(f"  X range: [{np.min(points[:, 0]):.2f}, {np.max(points[:, 0]):.2f}]")
    print(f"  Y range: [{np.min(points[:, 1]):.2f}, {np.max(points[:, 1]):.2f}]")
    print(f"  Z range: [{np.min(points[:, 2]):.2f}, {np.max(points[:, 2]):.2f}]")
    
    # Publish point cloud
    print("\nPublishing point cloud...")
    node = PointCloudPublisher(points)
    
    print("\nPoint cloud published!")
    print("The parameterization node should now process it.")
    print("\nYou can now test the services:")
    print("  ros2 service call /parameterization/get_uv_bounds parameterization/srv/GetUVBounds")
    print("  ros2 service call /parameterization/interpolate parameterization/srv/InterpolatePoint \"{u: [0.0], v: [0.0]}\"")
    
    # Keep node alive for a bit
    rclpy.spin_once(node, timeout_sec=2.0)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
