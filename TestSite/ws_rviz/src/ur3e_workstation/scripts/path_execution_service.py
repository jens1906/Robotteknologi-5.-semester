#!/usr/bin/env python3
"""
Path execution service for the Path Follower Node.
Provides services to execute Cartesian path or point-to-point motion.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from path_follower import PathFollowerNode


class PathExecutionService(Node):
    """Service node to trigger path execution after visualization."""
    
    def __init__(self, path_follower):
        super().__init__('path_execution_service')
        self.path_follower = path_follower
        
        # Create services
        self.cartesian_srv = self.create_service(
            Trigger,
            '/execute_cartesian_path',
            self.execute_cartesian_callback
        )
        
        self.point_to_point_srv = self.create_service(
            Trigger,
            '/execute_point_to_point',
            self.execute_point_to_point_callback
        )
        
        self.get_logger().info('Path execution services ready!')
        self.get_logger().info('  - /execute_cartesian_path')
        self.get_logger().info('  - /execute_point_to_point')
    
    def execute_cartesian_callback(self, request, response):
        """Execute the path using Cartesian planning."""
        self.get_logger().info('Cartesian path execution requested')
        
        if not self.path_follower.path_received:
            response.success = False
            response.message = 'No path received yet!'
            return response
        
        success = self.path_follower.execute_cartesian_path()
        response.success = success
        response.message = 'Path executed successfully!' if success else 'Path execution failed!'
        
        return response
    
    def execute_point_to_point_callback(self, request, response):
        """Execute the path point-to-point."""
        self.get_logger().info('Point-to-point execution requested')
        
        if not self.path_follower.path_received:
            response.success = False
            response.message = 'No path received yet!'
            return response
        
        success = self.path_follower.execute_point_to_point()
        response.success = success
        response.message = 'Path executed successfully!' if success else 'Path execution failed!'
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create path follower node
        path_follower = PathFollowerNode()
        
        # Create service node
        service_node = PathExecutionService(path_follower)
        
        # Use MultiThreadedExecutor to handle both nodes
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(path_follower)
        executor.add_node(service_node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            path_follower.destroy_node()
            service_node.destroy_node()
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
