"""
Example script demonstrating how to use the parameterization services.

This script shows how to:
1. Wait for the parameterization node to be ready
2. Get UV bounds
3. Query interpolation at specific (u,v) coordinates
4. Generate a scanning path
"""

import rclpy
from rclpy.node import Node
from parameterization.srv import InterpolatePoint, GetUVBounds
from parameterization.msg import ParameterizationStatus, UVPoint
import numpy as np
import time


class ParameterizationClient(Node):
    """Client node for parameterization services"""
    
    def __init__(self):
        super().__init__('parameterization_client')
        
        # Create service clients
        self.interpolate_client = self.create_client(
            InterpolatePoint,
            '/parameterization/interpolate'
        )
        
        self.bounds_client = self.create_client(
            GetUVBounds,
            '/parameterization/get_uv_bounds'
        )
        
        # Subscribe to status
        self.status_sub = self.create_subscription(
            ParameterizationStatus,
            '/parameterization/status',
            self.status_callback,
            10
        )
        
        self.is_ready = False
        self.latest_status = None
        
        # Wait for services
        self.get_logger().info('Waiting for parameterization services...')
        self.interpolate_client.wait_for_service(timeout_sec=10.0)
        self.bounds_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Services available!')
        
    def status_callback(self, msg):
        """Callback for status messages"""
        self.is_ready = msg.is_ready
        self.latest_status = msg
        
    def wait_for_ready(self, timeout=30.0):
        """Wait for parameterization to be ready"""
        self.get_logger().info('Waiting for parameterization to be ready...')
        start_time = time.time()
        
        while not self.is_ready and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.is_ready:
            self.get_logger().info('Parameterization is ready!')
            if self.latest_status:
                self.get_logger().info(f'  Points: {self.latest_status.num_points}')
                self.get_logger().info(f'  RMSE: {self.latest_status.rmse:.6f}')
            return True
        else:
            self.get_logger().error('Timeout waiting for parameterization')
            return False
    
    def get_uv_bounds(self):
        """Get UV parameter space bounds"""
        request = GetUVBounds.Request()
        future = self.bounds_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            return {
                'u_min': response.u_min,
                'u_max': response.u_max,
                'v_min': response.v_min,
                'v_max': response.v_max
            }
        else:
            self.get_logger().error(f'Failed to get bounds: {response.message}')
            return None
    
    def interpolate_points(self, u_array, v_array):
        """
        Interpolate 3D points from (u,v) coordinates.
        
        Args:
            u_array: List of u coordinates
            v_array: List of v coordinates
            
        Returns:
            List of (x, y, z) tuples, or None on failure
        """
        request = InterpolatePoint.Request()
        
        # Create list of UVPoint messages
        request.uv_points = []
        for u, v in zip(u_array, v_array):
            uv_pt = UVPoint()
            uv_pt.u = float(u)
            uv_pt.v = float(v)
            request.uv_points.append(uv_pt)
        
        future = self.interpolate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            points = [(p.x, p.y, p.z) for p in response.points]
            return points
        else:
            self.get_logger().error(f'Interpolation failed: {response.message}')
            return None
    
    def generate_scanning_path(self, num_passes=5, points_per_pass=10):
        """
        Generate a simple scanning path in UV space.
        
        Args:
            num_passes: Number of scanning passes
            points_per_pass: Number of points per pass
            
        Returns:
            List of waypoints as (x, y, z) tuples
        """
        # Get UV bounds
        bounds = self.get_uv_bounds()
        if bounds is None:
            return None
        
        self.get_logger().info(f'UV bounds: U=[{bounds["u_min"]:.3f}, {bounds["u_max"]:.3f}], '
                             f'V=[{bounds["v_min"]:.3f}, {bounds["v_max"]:.3f}]')
        
        # Generate UV path
        u_values = []
        v_values = []
        
        for i in range(num_passes):
            v = bounds['v_min'] + (bounds['v_max'] - bounds['v_min']) * i / (num_passes - 1)
            u_line = np.linspace(bounds['u_min'], bounds['u_max'], points_per_pass)
            
            # Alternate direction for boustrophedon pattern
            if i % 2 == 1:
                u_line = u_line[::-1]
            
            u_values.extend(u_line)
            v_values.extend([v] * points_per_pass)
        
        self.get_logger().info(f'Generated {len(u_values)} waypoints in UV space')
        
        # Interpolate to get 3D positions
        positions = self.interpolate_points(u_values, v_values)
        if positions is None:
            return None
        
        # Create waypoints
        waypoints = []
        for pos in positions:
            waypoints.append({
                'position': pos
            })
        
        return waypoints


def main():
    """Main function"""
    rclpy.init()
    
    print("=" * 70)
    print("  Parameterization Service Client Example")
    print("=" * 70)
    
    client = ParameterizationClient()
    
    # Wait for parameterization to be ready
    if not client.wait_for_ready(timeout=30.0):
        print("\nError: Parameterization not ready. Make sure:")
        print("  1. The parameterization node is running")
        print("  2. A point cloud has been published to /point_cloud")
        client.destroy_node()
        rclpy.shutdown()
        return
    
    # Get UV bounds
    print("\n" + "=" * 70)
    print("  Getting UV Bounds")
    print("=" * 70)
    bounds = client.get_uv_bounds()
    if bounds:
        print(f"\nUV parameter space bounds:")
        print(f"  U: [{bounds['u_min']:.3f}, {bounds['u_max']:.3f}]")
        print(f"  V: [{bounds['v_min']:.3f}, {bounds['v_max']:.3f}]")
    
    # Test single point interpolation
    print("\n" + "=" * 70)
    print("  Testing Single Point Interpolation")
    print("=" * 70)
    if bounds:
        u_test = (bounds['u_min'] + bounds['u_max']) / 2
        v_test = (bounds['v_min'] + bounds['v_max']) / 2
        
        print(f"\nQuerying at (u={u_test:.3f}, v={v_test:.3f})")
        
        positions = client.interpolate_points([u_test], [v_test])
        if positions:
            pos = positions[0]
            print(f"  Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    
    # Generate scanning path
    print("\n" + "=" * 70)
    print("  Generating Scanning Path")
    print("=" * 70)
    
    num_passes = 5
    points_per_pass = 10
    
    print(f"\nGenerating path with {num_passes} passes, {points_per_pass} points per pass...")
    waypoints = client.generate_scanning_path(num_passes, points_per_pass)
    
    if waypoints:
        print(f"\nGenerated {len(waypoints)} waypoints")
        
        # Calculate path length
        path_length = 0.0
        for i in range(1, len(waypoints)):
            p1 = np.array(waypoints[i-1]['position'])
            p2 = np.array(waypoints[i]['position'])
            path_length += np.linalg.norm(p2 - p1)
        
        print(f"Total path length: {path_length:.2f} units")
        
        # Show first and last waypoints
        print("\nFirst waypoint:")
        print(f"  Position: {waypoints[0]['position']}")
        
        print("\nLast waypoint:")
        print(f"  Position: {waypoints[-1]['position']}")
        
        # Save to file (optional)
        save_choice = input("\nSave waypoints to file? (y/n) [n]: ").strip().lower()
        if save_choice == 'y':
            import json
            filename = 'scanning_path.json'
            with open(filename, 'w') as f:
                json.dump(waypoints, f, indent=2)
            print(f"Saved to {filename}")
    
    print("\n" + "=" * 70)
    print("  Example Complete!")
    print("=" * 70)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
