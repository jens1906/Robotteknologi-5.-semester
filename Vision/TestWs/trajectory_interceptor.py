#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.action import ActionClient
import time

class TrajectoryInterceptor(Node):
    def __init__(self):
        super().__init__('trajectory_interceptor')
        
        # Create action server to intercept MoveIt requests
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory_interceptor',
            self.execute_callback
        )
        
        # Create action client to forward to real controller
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info("Trajectory interceptor started")
        self.get_logger().info("Waiting for real controller...")
        self.action_client.wait_for_server()
        self.get_logger().info("Connected to real controller")
    
    def execute_callback(self, goal_handle):
        self.get_logger().info("=== INTERCEPTED TRAJECTORY FROM MOVEIT ===")
        
        request = goal_handle.request
        trajectory = request.trajectory
        
        # Log trajectory details
        self.get_logger().info(f"Joint names: {trajectory.joint_names}")
        self.get_logger().info(f"Number of points: {len(trajectory.points)}")
        
        if trajectory.points:
            first_point = trajectory.points[0]
            last_point = trajectory.points[-1]
            
            self.get_logger().info(f"First point positions: {first_point.positions}")
            self.get_logger().info(f"First point time: {first_point.time_from_start}")
            self.get_logger().info(f"Last point positions: {last_point.positions}")
            self.get_logger().info(f"Last point time: {last_point.time_from_start}")
        
        # Log goal tolerances
        self.get_logger().info(f"Goal tolerances ({len(request.goal_tolerance)} items):")
        for tol in request.goal_tolerance:
            self.get_logger().info(f"  {tol.name}: pos={tol.position}, vel={tol.velocity}, acc={tol.acceleration}")
        
        # Log path tolerances  
        self.get_logger().info(f"Path tolerances ({len(request.path_tolerance)} items):")
        for tol in request.path_tolerance:
            self.get_logger().info(f"  {tol.name}: pos={tol.position}, vel={tol.velocity}, acc={tol.acceleration}")
            
        # Log goal time tolerance
        goal_time_sec = request.goal_time_tolerance.sec + request.goal_time_tolerance.nanosec / 1e9
        self.get_logger().info(f"Goal time tolerance: {goal_time_sec} seconds")
        
        self.get_logger().info("=== FORWARDING TO REAL CONTROLLER ===")
        
        # Forward the request to the real controller
        future = self.action_client.send_goal_async(request)
        
        def handle_response(future_response):
            goal_handle_real = future_response.result()
            if not goal_handle_real.accepted:
                self.get_logger().error("Real controller REJECTED the goal")
                goal_handle.abort()
                return
            
            self.get_logger().info("Real controller ACCEPTED the goal")
            
            # Wait for result
            result_future = goal_handle_real.get_result_async()
            
            def handle_result(future_result):
                result = future_result.result().result
                if result.error_code == 0:
                    self.get_logger().info("Real controller SUCCEEDED")
                    goal_handle.succeed()
                else:
                    self.get_logger().error(f"Real controller FAILED: {result.error_string}")
                    goal_handle.abort()
            
            result_future.add_done_callback(handle_result)
        
        future.add_done_callback(handle_response)
        
        # Return the goal handle (don't return result yet)
        return goal_handle

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryInterceptor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()