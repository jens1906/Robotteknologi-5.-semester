"""
Test script for Path Planning Node

This script:
1. Imports and runs the PathPlanner node
2. Publishes mock parameterization data via topics (no custom messages needed)
3. Publishes ready status, UV bounds, and boundary points
4. Publishes tool size information
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray, Bool

# Import the PathPlanner class
from path_planning_node import PathPlanner


class PathPlanningTestPublisher(Node):
    """Test node that publishes inputs for the PathPlanner"""
    
    def __init__(self):
        super().__init__('path_planning_test_publisher')
        
        # Publishers
        self.ready_pub = self.create_publisher(
            Bool,
            '/parameterization/ready',
            10
        )
        
        self.bounds_pub = self.create_publisher(
            Float64MultiArray,
            '/parameterization/uv_bounds',
            10
        )
        
        self.boundary_pub = self.create_publisher(
            Float64MultiArray,
            '/parameterization/uv_boundary',
            10
        )
        
        self.tool_size_pub = self.create_publisher(
            Float64MultiArray,
            '/corrosion/tool_size',
            10
        )
        
        # Test data configuration
        self.u_min = 0.0
        self.u_max = 200.0
        self.v_min = 0.0
        self.v_max = 150.0
        self.tool_size = 10.0  # mm
        
        # Create boundary points (rectangular for simplicity, or custom polygon)
        self.boundary_points = self._create_boundary_points()
        
        # Timer to publish status and tool size
        self.timer = self.create_timer(2.0, self.publish_test_data)
        
        self.get_logger().info('Path Planning Test Publisher initialized')
        self.get_logger().info(f'  UV bounds: u=[{self.u_min}, {self.u_max}], v=[{self.v_min}, {self.v_max}]')
        self.get_logger().info(f'  Tool size: {self.tool_size} mm')
        self.get_logger().info(f'  Boundary points: {len(self.boundary_points)}')
    
    
    def _create_boundary_points(self):
        """Create boundary points for the UV space.
        
        You can customize this to create different shapes:
        - Rectangle: 4 corner points
        - Irregular polygon: any number of points
        - Circle/ellipse approximation: many points in a circular pattern
        """
        # Example 1: Simple rectangle
        # return [
        #     [self.u_min, self.v_min],
        #     [self.u_max, self.v_min],
        #     [self.u_max, self.v_max],
        #     [self.u_min, self.v_max]
        # ]
        
        # Example 2: Irregular polygon (more realistic corrosion area)
        return [
            [20.0, 20.0],
            [180.0, 30.0],
            [190.0, 70.0],
            [170.0, 130.0],
            [100.0, 140.0],
            [30.0, 120.0],
            [10.0, 60.0]
        ]
        
        # Example 3: Circular/elliptical boundary
        # n_points = 20
        # theta = np.linspace(0, 2*np.pi, n_points, endpoint=False)
        # u_center = (self.u_min + self.u_max) / 2
        # v_center = (self.v_min + self.v_max) / 2
        # u_radius = (self.u_max - self.u_min) / 2 * 0.8
        # v_radius = (self.v_max - self.v_min) / 2 * 0.8
        # return [[u_center + u_radius * np.cos(t), v_center + v_radius * np.sin(t)] 
        #         for t in theta]
    
    
    def publish_test_data(self):
        """Publish parameterization ready status, UV bounds, boundary, and tool size."""
        # Publish ready status
        ready_msg = Bool()
        ready_msg.data = True
        self.ready_pub.publish(ready_msg)
        self.get_logger().info('Published parameterization ready=True')
        
        # Publish UV bounds [u_min, u_max, v_min, v_max]
        bounds_msg = Float64MultiArray()
        bounds_msg.data = [self.u_min, self.u_max, self.v_min, self.v_max]
        self.bounds_pub.publish(bounds_msg)
        self.get_logger().info(f'Published UV bounds: [{self.u_min}, {self.u_max}, {self.v_min}, {self.v_max}]')
        
        # Publish boundary points (flattened Nx2)
        boundary_msg = Float64MultiArray()
        boundary_msg.data = np.array(self.boundary_points).flatten().tolist()
        self.boundary_pub.publish(boundary_msg)
        self.get_logger().info(f'Published {len(self.boundary_points)} boundary points')
        
        # Publish tool size
        tool_msg = Float64MultiArray()
        tool_msg.data = [0.0, self.tool_size]  # data[1] is tool size
        self.tool_size_pub.publish(tool_msg)
        self.get_logger().info(f'Published tool size: {self.tool_size}')


def main():
    """Main function to run test publisher and PathPlanner together."""
    try:
        rclpy.init()
        
        # Create both nodes
        test_publisher = PathPlanningTestPublisher()
        path_planner = PathPlanner()
        
        # Use MultiThreadedExecutor to run both nodes
        executor = MultiThreadedExecutor()
        executor.add_node(test_publisher)
        executor.add_node(path_planner)
        
        print('\n' + '='*60)
        print('Path Planning Test Environment')
        print('='*60)
        print('Running PathPlanner with mock input data...')
        print('Press Ctrl+C to stop')
        print('='*60 + '\n')
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            print('\nShutting down...')
        finally:
            executor.shutdown()
            test_publisher.destroy_node()
            path_planner.destroy_node()
            rclpy.shutdown()
    
    except Exception as e:
        print(f"Test - Error in main(): {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
