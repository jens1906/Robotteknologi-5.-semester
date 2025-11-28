#!/usr/bin/env python3
"""
Test the controller with a properly timed trajectory to check if it's a controller issue
or a MoveIt trajectory generation issue.
"""

import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration

class SimpleTrajectoryTester(Node):
    def __init__(self):
        super().__init__('simple_trajectory_tester')
        
        # Action client to send trajectory directly to controller
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Simple trajectory tester started')
        self.get_logger().info('Waiting for controller...')
        
        if self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('Controller found! Sending test trajectory...')
            self.send_test_trajectory()
        else:
            self.get_logger().error('Controller not found!')
    
    def send_test_trajectory(self):
        """Send a simple trajectory with proper timing"""
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
        
        # Point 1: Start position (current position approximation)
        point1 = JointTrajectoryPoint()
        point1.positions = [1.57, -1.57, 1.57, -1.57, 0.0, 0.0]  # Roughly current position
        point1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point1.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1 seconds
        
        # Point 2: Slightly different position
        point2 = JointTrajectoryPoint()
        point2.positions = [1.6, -1.5, 1.6, -1.5, 0.1, 0.1]  # Small change
        point2.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point2.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point2.time_from_start = Duration(sec=2, nanosec=0)  # 2 seconds total
        
        trajectory.points = [point1, point2]
        goal.trajectory = trajectory
        
        # Set very permissive tolerances like our successful test
        goal.goal_tolerance = []
        goal.path_tolerance = []
        
        # Set goal time tolerance
        goal.goal_time_tolerance = Duration(sec=10, nanosec=0)  # 10 seconds
        
        self.get_logger().info(f'Sending trajectory with {len(trajectory.points)} points')
        self.get_logger().info(f'Point 0 time: {point1.time_from_start.sec}.{point1.time_from_start.nanosec}')
        self.get_logger().info(f'Point 1 time: {point2.time_from_start.sec}.{point2.time_from_start.nanosec}')
        
        # Send the goal
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_callback)
    
    def goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal REJECTED by controller!')
            self.get_logger().error('This confirms the controller is rejecting trajectories directly')
        else:
            self.get_logger().info('Trajectory goal ACCEPTED by controller!')
            self.get_logger().info('Controller issue resolved - problem is in MoveIt trajectory generation')
            
            # Get result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result()
        if result.result.error_code == 0:
            self.get_logger().info('Trajectory executed SUCCESSFULLY!')
        else:
            self.get_logger().error(f'Trajectory execution failed with error: {result.result.error_code}')

def main():
    rclpy.init()
    tester = SimpleTrajectoryTester()
    
    try:
        # Spin for enough time to get feedback
        for i in range(100):  # 10 seconds total
            rclpy.spin_once(tester, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()