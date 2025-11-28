#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
import time

class SimpleTrajectoryTester(Node):
    def __init__(self):
        super().__init__('trajectory_tester')
        
        # Create action client
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info("Waiting for action server...")
        self.action_client.wait_for_server()
        self.get_logger().info("Action server found!")
        
    def send_simple_trajectory(self):
        # Create a very simple trajectory - just move slightly from current position
        goal = FollowJointTrajectory.Goal()
        
        # Set up the trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Create a simple point - current position + small offset
        point = JointTrajectoryPoint()
        # These are approximate current positions based on what we saw earlier
        point.positions = [1.57, -1.57, 1.57, -1.57, 0.0, 0.0]  # Roughly 90° positions
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 3  # 3 seconds to reach
        point.time_from_start.nanosec = 0
        
        trajectory.points = [point]
        goal.trajectory = trajectory
        
        # Set very permissive tolerances
        from control_msgs.msg import JointTolerance
        
        goal.goal_tolerance = []
        for joint_name in trajectory.joint_names:
            tol = JointTolerance()
            tol.name = joint_name
            tol.position = 0.5  # Very permissive: 0.5 radians ≈ 29°
            tol.velocity = 1.0
            tol.acceleration = 1.0
            goal.goal_tolerance.append(tol)
            
        # Set goal time tolerance
        goal.goal_time_tolerance.sec = 10
        goal.goal_time_tolerance.nanosec = 0
        
        self.get_logger().info("Sending trajectory goal...")
        
        # Send the goal
        future = self.action_client.send_goal_async(goal)
        
        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected!")
                return
            
            self.get_logger().info("Goal accepted! Waiting for result...")
            
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
        
        future.add_done_callback(goal_response_callback)
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Trajectory execution finished with error code: {result.error_code}")
        if result.error_code == 0:
            self.get_logger().info("SUCCESS: Trajectory executed successfully!")
        else:
            self.get_logger().error(f"FAILED: Error string: {result.error_string}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectoryTester()
    
    # Wait a bit for everything to initialize
    time.sleep(2)
    
    # Send the test trajectory
    node.send_simple_trajectory()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()