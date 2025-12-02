#!/usr/bin/env python3
"""
Follow a scanning path using MoveIt's Cartesian path planning.
Executes the path exactly as provided without iteration or modification.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
import sys
import time

class CartesianPathFollower(Node):
    def __init__(self):
        super().__init__('cartesian_path_follower')
        
        self.get_logger().info('='*60)
        self.get_logger().info('Cartesian Path Follower - Single Run Mode')
        self.get_logger().info('='*60)
        
        self.waypoints = []
        self.path_received = False
        
        # Service client for Cartesian path
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        
        if not self.cartesian_path_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Cartesian path service not available!')
            sys.exit(1)
        
        # Action client for trajectory execution
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('ExecuteTrajectory action not available!')
            sys.exit(1)

        # Action client for moving to start (MoveGroup)
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action not available!')
            sys.exit(1)
        
        # Subscribe to path topic
        self.path_sub = self.create_subscription(
            PoseArray,
            '/tool_orientation/path',
            self.path_callback,
            10
        )
        
        self.get_logger().info('Waiting for path on /tool_orientation/path...')

    def path_callback(self, msg):
        if self.path_received:
            return # Only take the first one or wait for restart
            
        self.get_logger().info(f'Received path with {len(msg.poses)} points')
        self.waypoints = msg.poses
        self.path_received = True
        
    def move_to_start_pose(self, pose):
        """Move to the first waypoint using standard PTP planning."""
        self.get_logger().info('Moving to start pose...')
        
        goal = MoveGroup.Goal()
        goal.request.group_name = 'ur_manipulator'
        goal.request.num_planning_attempts = 20
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.planner_id = "RRTConnectkConfigDefault"
        
        # Create constraints
        c = Constraints()
        
        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = 'world'
        pc.link_name = 'tool0'
        pc.weight = 1.0
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01] # 1cm tolerance
        bv.primitives.append(sp)
        p_pose = Pose()
        p_pose.position = pose.position
        p_pose.orientation.w = 1.0
        bv.primitive_poses.append(p_pose)
        pc.constraint_region = bv
        c.position_constraints.append(pc)
        
        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = 'world'
        oc.link_name = 'tool0'
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        
        goal.request.goal_constraints.append(c)
        
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.done() or not future.result().accepted:
            self.get_logger().error('Start move rejected')
            return False
            
        handle = future.result()
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        result = res_future.result().result
        if result.error_code.val == 1: # SUCCESS
            self.get_logger().info('✓ Reached start pose')
            return True
        else:
            self.get_logger().error(f'✗ Failed to reach start pose: {result.error_code.val}')
            return False

    def execute_path(self):
        if not self.waypoints:
            self.get_logger().error('No waypoints received')
            return

        # 1. Move to start
        if not self.move_to_start_pose(self.waypoints[0]):
            self.get_logger().error('Aborting: Could not reach start point')
            return

        # 2. Compute Cartesian Path
        self.get_logger().info(f'Computing Cartesian path for {len(self.waypoints)} waypoints...')
        
        request = GetCartesianPath.Request()
        request.header.frame_id = 'world'
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = 'ur_manipulator'
        request.link_name = 'tool0'
        request.waypoints = self.waypoints
        request.max_step = 0.01  # 1cm interpolation
        request.jump_threshold = 0.0  # Disable jump check (permissive)
        request.avoid_collisions = True
        
        future = self.cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.done():
            self.get_logger().error('Cartesian path service failed')
            return
            
        response = future.result()
        fraction = response.fraction
        
        # Calculate success
        total_points = len(self.waypoints)
        # fraction is (points_computed - 1) / (total_points - 1) roughly, or just percentage of path length
        # We'll estimate successful points
        successful_count = int(fraction * total_points)
        if successful_count < 1 and fraction > 0:
            successful_count = 1
            
        self.get_logger().info(f'Path computation result: {fraction*100:.1f}% of path resolved')
        
        # 3. Execute
        if response.solution.joint_trajectory.points:
            self.get_logger().info(f'Executing trajectory ({len(response.solution.joint_trajectory.points)} points)...')
            
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = response.solution
            
            goal_future = self.execute_trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, goal_future)
            
            if not goal_future.done() or not goal_future.result().accepted:
                self.get_logger().error('Trajectory execution rejected')
                return
                
            handle = goal_future.result()
            res_future = handle.get_result_async()
            
            # Wait for execution
            self.get_logger().info('Waiting for execution to complete...')
            rclpy.spin_until_future_complete(self, res_future)
            
            exec_result = res_future.result().result
            if exec_result.error_code.val == 1: # SUCCESS
                self.get_logger().info('✓ Execution complete')
            else:
                self.get_logger().error(f'✗ Execution failed: {exec_result.error_code.val}')
        else:
            self.get_logger().error('No trajectory generated (fraction too low?)')

        # 4. Final Report
        print("\n" + "="*60)
        print("FINAL EXECUTION REPORT")
        print("="*60)
        
        for i in range(total_points):
            if i < successful_count:
                print(f"Point {i+1}: SUCCESS")
            else:
                print(f"Point {i+1}: FAILED")
                
        success_rate = (successful_count / total_points) * 100
        print("-" * 60)
        print(f"Total Success Rate: {success_rate:.1f}% ({successful_count}/{total_points})")
        print("="*60 + "\n")


def main():
    rclpy.init()
    node = CartesianPathFollower()
    
    # Wait for path
    while not node.path_received:
        rclpy.spin_once(node, timeout_sec=0.5)
        
    # Run once
    node.execute_path()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
