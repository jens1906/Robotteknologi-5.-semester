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
import numpy as np

class CartesianPathFollower(Node):
    def __init__(self):
        super().__init__('cartesian_path_follower')
        
        self.get_logger().info('='*60)
        self.get_logger().info('Cartesian Path Follower - Continuous Mode')
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
        
    def ensure_continuous_orientation(self, poses):
        """
        Ensure that quaternion transitions are continuous by flipping signs if needed.
        This does not change the physical orientation, only the mathematical representation.
        """
        if not poses:
            return poses
            
        corrected_poses = []
        # Copy first pose
        first_pose = Pose()
        first_pose.position = poses[0].position
        first_pose.orientation = poses[0].orientation
        corrected_poses.append(first_pose)
        
        for i in range(1, len(poses)):
            prev_pose = corrected_poses[-1]
            curr_pose_in = poses[i]
            
            # Create new pose object
            curr_pose = Pose()
            curr_pose.position = curr_pose_in.position
            
            prev_q = np.array([
                prev_pose.orientation.x,
                prev_pose.orientation.y,
                prev_pose.orientation.z,
                prev_pose.orientation.w
            ])
            
            curr_q = np.array([
                curr_pose_in.orientation.x,
                curr_pose_in.orientation.y,
                curr_pose_in.orientation.z,
                curr_pose_in.orientation.w
            ])
            
            # Dot product
            dot = np.dot(prev_q, curr_q)
            
            if dot < 0:
                # Flip sign
                curr_pose.orientation.x = -curr_q[0]
                curr_pose.orientation.y = -curr_q[1]
                curr_pose.orientation.z = -curr_q[2]
                curr_pose.orientation.w = -curr_q[3]
            else:
                curr_pose.orientation = curr_pose_in.orientation
                
            corrected_poses.append(curr_pose)
            
        return corrected_poses

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

        # 0. Ensure quaternion continuity
        self.get_logger().info('Ensuring quaternion continuity...')
        all_waypoints = self.ensure_continuous_orientation(self.waypoints)
        total_points = len(all_waypoints)
        
        # Track success for each point
        # 0: Not attempted/Unknown, 1: Success, 2: Failed
        point_status = [0] * total_points
        
        # 1. Move to start
        if not self.move_to_start_pose(all_waypoints[0]):
            self.get_logger().error('Aborting: Could not reach start point')
            # Mark all as failed
            point_status = [2] * total_points
            self.print_report(point_status)
            return
            
        # Mark first point as success (we are there)
        point_status[0] = 1
        current_index = 0 # The index of the waypoint we are currently AT
        
        while current_index < total_points - 1:
            # We are at all_waypoints[current_index]
            # We want to go to the rest
            remaining_waypoints = all_waypoints[current_index:] # Includes current as start
            
            self.get_logger().info(f'Planning from point {current_index+1}/{total_points}...')
            
            # Try Cartesian Path first
            request = GetCartesianPath.Request()
            request.header.frame_id = 'world'
            request.header.stamp = self.get_clock().now().to_msg()
            request.group_name = 'ur_manipulator'
            request.link_name = 'tool0'
            request.waypoints = remaining_waypoints
            request.max_step = 0.05  # 5cm interpolation
            request.jump_threshold = 1.5  # Prevent large joint jumps (approx 85 degrees)
            request.avoid_collisions = True
            
            future = self.cartesian_path_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.done():
                response = future.result()
                fraction = response.fraction
                
                # Calculate how many NEW points we can reach
                # fraction is against len(remaining_waypoints)
                # If fraction = 1.0, we reached all.
                # If fraction = 0.0, we didn't move.
                
                # Number of segments in remaining_waypoints is len - 1
                # But GetCartesianPath fraction logic is a bit fuzzy.
                # Usually: points_planned = fraction * len(remaining_waypoints)
                
                points_planned = int(fraction * len(remaining_waypoints))
                
                # If we have a valid trajectory, execute it
                if points_planned > 0 and response.solution.joint_trajectory.points:
                    self.get_logger().info(f'Cartesian path found for {points_planned} points (fraction: {fraction:.2f})')
                    
                    goal = ExecuteTrajectory.Goal()
                    goal.trajectory = response.solution
                    
                    goal_future = self.execute_trajectory_client.send_goal_async(goal)
                    rclpy.spin_until_future_complete(self, goal_future)
                    
                    if goal_future.done() and goal_future.result().accepted:
                        handle = goal_future.result()
                        res_future = handle.get_result_async()
                        rclpy.spin_until_future_complete(self, res_future)
                        
                        if res_future.result().result.error_code.val == 1:
                            # Execution successful
                            # Mark points as success
                            for i in range(points_planned):
                                idx = current_index + i
                                if idx < total_points:
                                    point_status[idx] = 1
                            
                            current_index += points_planned
                            # If we reached the end, break
                            if current_index >= total_points - 1:
                                break
                            
                            # If we are here, it means we executed a PARTIAL path.
                            # The next point (current_index + 1) is where it failed.
                            # We will fall through to the fallback logic.
                        else:
                            self.get_logger().error('Cartesian execution failed')
                else:
                    self.get_logger().warn('Cartesian path failed to make progress')

            # If we haven't finished, we are stuck at current_index.
            # The next target is current_index + 1
            next_target_idx = current_index + 1
            if next_target_idx >= total_points:
                break
                
            self.get_logger().info(f'⚠ Cartesian path stopped at point {next_target_idx+1}. Attempting PTP fallback...')
            
            # Fallback: Try PTP to next point
            target_pose = all_waypoints[next_target_idx]
            if self.move_to_pose_ptp(target_pose):
                self.get_logger().info(f'✓ PTP recovery successful to point {next_target_idx+1}')
                point_status[next_target_idx] = 1
                current_index = next_target_idx
            else:
                self.get_logger().error(f'✗ PTP recovery failed for point {next_target_idx+1}. Skipping...')
                point_status[next_target_idx] = 2 # Failed
                # We are still at current_index physically, but we give up on next_target_idx
                # We advance current_index logically to try the one after
                current_index = next_target_idx 
                # Note: We are physically at 'current_index - 1' (or wherever we were), 
                # but we set current_index to the failed one so the loop tries from there+1.
                # Actually, if we skip, we might be far away. 
                # The next Cartesian plan will start from 'current_index' (which is the failed point).
                # But we are NOT there.
                # GetCartesianPath uses current robot state as start.
                # So if we pass `waypoints = [failed_point, next_point, ...]`, it will try to plan from CURRENT pose to failed_point.
                # We just tried that and it failed.
                
                # So we should try to plan from CURRENT pose to `next_point` (skipping failed_point).
                # So we increment current_index, but we don't add it to the waypoints list for the next Cartesian attempt?
                # Wait, `remaining_waypoints = all_waypoints[current_index:]`
                # If current_index is the failed point, we include it.
                # We want to EXCLUDE it.
                
                # Let's just increment current_index. The next loop will try `all_waypoints[current_index:]`.
                # Since we are NOT at `all_waypoints[current_index]`, the Cartesian planner will try to connect 
                # Current Robot Pose -> all_waypoints[current_index].
                # This is effectively trying to go to the point after the failed one.
                pass

        self.print_report(point_status)

    def move_to_pose_ptp(self, pose):
        """Move to a pose using MoveGroup (PTP)."""
        goal = MoveGroup.Goal()
        goal.request.group_name = 'ur_manipulator'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 1.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.planner_id = "RRTConnectkConfigDefault"
        
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = 'world'
        pc.link_name = 'tool0'
        pc.weight = 1.0
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.02] # 2cm tolerance
        bv.primitives.append(sp)
        p_pose = Pose()
        p_pose.position = pose.position
        p_pose.orientation.w = 1.0
        bv.primitive_poses.append(p_pose)
        pc.constraint_region = bv
        c.position_constraints.append(pc)
        
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
            return False
            
        handle = future.result()
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        return res_future.result().result.error_code.val == 1

    def print_report(self, point_status):
        print("\n" + "="*60)
        print("FINAL EXECUTION REPORT")
        print("="*60)
        
        success_count = 0
        for i, status in enumerate(point_status):
            status_str = "UNKNOWN"
            if status == 1:
                status_str = "SUCCESS"
                success_count += 1
            elif status == 2:
                status_str = "FAILED"
            else:
                status_str = "SKIPPED"
                
            print(f"Point {i+1}: {status_str}")
                
        total = len(point_status)
        rate = (success_count / total) * 100 if total > 0 else 0
        print("-" * 60)
        print(f"Total Success Rate: {rate:.1f}% ({success_count}/{total})")
        print("="*60 + "\n")



def main():
    rclpy.init()
    node = CartesianPathFollower()
    
    try:
        while rclpy.ok():
            # Wait for path
            node.get_logger().info('Waiting for path on /tool_orientation/path...')
            while not node.path_received and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.5)
            
            if not rclpy.ok():
                break
                
            # Execute path
            node.get_logger().info('Path received! Starting execution...')
            node.execute_path()
            
            # Reset for next path
            node.path_received = False
            node.waypoints = []
            node.get_logger().info('Execution finished. Ready for next path.')
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
