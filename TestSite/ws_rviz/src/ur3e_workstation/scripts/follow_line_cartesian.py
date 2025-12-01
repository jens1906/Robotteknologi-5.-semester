#!/usr/bin/env python3
"""
Follow a line path using MoveIt's Cartesian path planning with collision avoidance.
Subscribes to /tool_orientation/xyz_rotation with format: [x,y,z,qx,qy,qz,qw, ...]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float64MultiArray
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory, RobotState
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys


class LinePathFollower(Node):
    """Follow a line path with Cartesian planning and collision avoidance."""
    
    def __init__(self):
        super().__init__('line_path_follower')
        
        self.get_logger().info('='*60)
        self.get_logger().info('Line Path Follower - Cartesian with Collision Avoidance')
        self.get_logger().info('='*60)
        
        # Create service client for Cartesian path
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        
        self.get_logger().info('Waiting for /compute_cartesian_path service...')
        if not self.cartesian_path_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Cartesian path service not available!')
            sys.exit(1)
        
        self.get_logger().info('✓ Cartesian path service ready')
        
        # Create action client for trajectory execution
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        
        self.get_logger().info('Waiting for /execute_trajectory action...')
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('ExecuteTrajectory action not available!')
            sys.exit(1)
        
        self.get_logger().info('✓ ExecuteTrajectory action ready')
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to path topic
        self.path_sub = self.create_subscription(
            PoseArray,
            '/tool_orientation/path',
            self.path_callback,
            10
        )
        
        self.current_joint_state = None
        self.waypoints = []
        self.path_received = False
        self.executing = False
        
        self.get_logger().info('Waiting for line path on /tool_orientation/path...')
        self.get_logger().info('Expected format: PoseArray with poses in base_link frame')
    
    def joint_state_callback(self, msg):
        """Store current joint state."""
        self.current_joint_state = msg
    
    def path_callback(self, msg):
        """Parse received PoseArray path."""
        if self.executing:
            self.get_logger().debug('Ignoring path update during execution')
            return
            
        num_waypoints = len(msg.poses)
        self.get_logger().info(f'Received PoseArray with {num_waypoints} poses')
        
        if num_waypoints == 0:
            self.get_logger().error('Received empty PoseArray')
            return
        
        # Use poses directly from PoseArray
        self.waypoints = list(msg.poses)
        
        # Log first and last poses for verification
        for i in [0, num_waypoints - 1]:
            pose = self.waypoints[i]
            self.get_logger().info(
                f'  Point {i}: pos=[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}] '
                f'quat=[{pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f}]'
            )
        
        self.path_received = True
        self.get_logger().info(f'✓ Successfully received {len(self.waypoints)} waypoints')
        self.get_logger().info('Ready to execute! Call: ros2 service call /execute_line_path std_srvs/srv/Trigger')
    
    def plan_cartesian_path(self):
        """Plan Cartesian path through all waypoints with collision avoidance."""
        if not self.waypoints:
            self.get_logger().error('No waypoints to plan!')
            return None
        
        if self.current_joint_state is None:
            self.get_logger().error('No joint state received!')
            return None
        
        self.get_logger().info(f'Planning Cartesian path through {len(self.waypoints)} waypoints...')
        
        # Create request
        request = GetCartesianPath.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'base_link'
        
        # Set start state (current joint state)
        request.start_state.joint_state = self.current_joint_state
        
        # Set group name
        request.group_name = 'ur_manipulator'
        request.link_name = 'tool0'
        
        # Set waypoints
        request.waypoints = self.waypoints
        
        # Cartesian planning parameters
        request.max_step = 0.01  # 1cm resolution
        request.jump_threshold = 0.0  # Disable jump detection
        request.avoid_collisions = True  # ENABLE collision avoidance
        
        self.get_logger().info('Sending Cartesian path request with collision avoidance enabled...')
        
        # Call service
        future = self.cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if not future.done():
            self.get_logger().error('Cartesian path planning timed out!')
            return None
        
        response = future.result()
        
        if response is None:
            self.get_logger().error('Cartesian path service call failed!')
            return None
        
        # Check fraction achieved
        fraction = response.fraction
        self.get_logger().info(f'Path planning result: {fraction*100:.1f}% of path achieved')
        
        if fraction < 0.95:
            self.get_logger().warn(f'Only {fraction*100:.1f}% of path could be planned!')
            self.get_logger().warn('This may indicate collision issues or unreachable waypoints')
        
        if fraction < 0.90:
            self.get_logger().error(f'Path planning failed - only {fraction*100:.1f}% achieved (minimum 90% required)')
            self.get_logger().error('The path likely has collisions or unreachable poses')
            self.get_logger().error('Please adjust the line position/orientation in line_simulator.py')
            return None
        
        # Log trajectory info
        traj = response.solution
        num_points = len(traj.joint_trajectory.points)
        if num_points > 0:
            duration = traj.joint_trajectory.points[-1].time_from_start.sec + \
                      traj.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9
            self.get_logger().info(f'Trajectory: {num_points} points, duration: {duration:.2f}s')
        
        return traj
    
    def execute_trajectory(self, trajectory):
        """Execute the planned trajectory."""
        self.get_logger().info('Executing trajectory...')
        
        # Create goal
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Send goal
        send_goal_future = self.execute_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory execution rejected!')
            return False
        
        self.get_logger().info('Trajectory accepted, executing...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('✓ Trajectory execution successful!')
            return True
        else:
            self.get_logger().error(f'Trajectory execution failed with error code: {result.result.error_code.val}')
            return False
    
    def execute_path_command(self):
        """Execute the stored path."""
        if not self.path_received:
            self.get_logger().error('No path received yet!')
            return False
        
        if self.executing:
            self.get_logger().warn('Already executing a path!')
            return False
        
        self.executing = True
        
        try:
            # Plan path
            trajectory = self.plan_cartesian_path()
            
            if trajectory is None:
                self.get_logger().error('Path planning failed!')
                return False
            
            # Execute
            success = self.execute_trajectory(trajectory)
            
            return success
            
        finally:
            self.executing = False


def main(args=None):
    rclpy.init(args=args)
    node = LinePathFollower()
    
    try:
        # Wait for joint states first
        node.get_logger().info('Waiting for joint states...')
        while node.current_joint_state is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info('✓ Joint states received')
        
        # Wait for path
        while not node.path_received:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # Execute path
        node.get_logger().info('Path received! Starting execution in 2 seconds...')
        import time
        time.sleep(2.0)
        
        success = node.execute_path_command()
        
        if success:
            node.get_logger().info('='*60)
            node.get_logger().info('PATH EXECUTION COMPLETED SUCCESSFULLY')
            node.get_logger().info('='*60)
        else:
            node.get_logger().error('='*60)
            node.get_logger().error('PATH EXECUTION FAILED')
            node.get_logger().error('='*60)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
