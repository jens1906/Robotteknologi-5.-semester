#!/usr/bin/env python3
"""
Follow a scanning path using MoveIt's Cartesian path planning.
Executes the path exactly as provided without iteration or modification.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.srv import GetCartesianPath, GetPositionIK, GetPositionFK
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, RobotState, PositionIKRequest, JointConstraint
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
import sys
import time
import numpy as np
import math
import random

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
        
        # Service client for IK
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

        # Service client for FK
        self.fk_client = self.create_client(
            GetPositionFK,
            '/compute_fk'
        )
        
        if not self.cartesian_path_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Cartesian path service not available!')
            sys.exit(1)
            
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available!')
            sys.exit(1)

        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('FK service not available!')
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
        
        # Subscribe to joint states for IK seeding
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.current_joint_state = None
        
        self.get_logger().info('Waiting for path on /tool_orientation/path...')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

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

    def get_ik(self, pose, seed_joint_state=None):
        """Get IK solution for a pose, optionally seeded."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur_manipulator'
        req.ik_request.pose_stamped.header.frame_id = 'world'
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.avoid_collisions = True
        
        # Seed
        if seed_joint_state:
            req.ik_request.robot_state.joint_state = seed_joint_state
        elif self.current_joint_state:
            req.ik_request.robot_state.joint_state = self.current_joint_state
        
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            result = future.result()
            if result.error_code.val == 1:
                return result.solution
        return None

    def get_fk(self, joint_state):
        """Get FK solution for a joint state."""
        req = GetPositionFK.Request()
        req.header.frame_id = 'world'
        req.fk_link_names = ['tool0']
        req.robot_state.joint_state = joint_state
        
        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            result = future.result()
            if result.error_code.val == 1:
                return result.pose_stamped[0].pose
        return None

    def move_to_joint_state(self, joint_state):
        """Move to a specific joint configuration using MoveGroup."""
        self.get_logger().info('Moving to start configuration...')
        
        goal = MoveGroup.Goal()
        goal.request.group_name = 'ur_manipulator'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 2.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.planner_id = "RRTConnectkConfigDefault"
        
        # Create joint constraints
        c = Constraints()
        for i, name in enumerate(joint_state.name):
            jc = PositionConstraint() # Wait, we need JointConstraint? No, MoveIt uses JointConstraint
            # But msg definition: JointConstraint
            pass 
        
        # Actually, simpler to just set goal_constraints with joint constraints
        # But constructing JointConstraint manually is tedious.
        # Let's use the 'joint_constraints' field of Constraints
        from moveit_msgs.msg import JointConstraint
        
        for i, name in enumerate(joint_state.name):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = joint_state.position[i]
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
            
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
            self.get_logger().info('✓ Reached start configuration')
            return True
        else:
            self.get_logger().error(f'✗ Failed to reach start configuration: {result.error_code.val}')
            return False

    def retime_trajectory(self, trajectory, speed=0.1):
        """
        Retime the trajectory to enforce a constant Cartesian speed.
        Uses FK to calculate actual distances.
        Filters points that are too close to ensure smooth execution.
        Enforces joint velocity limits to prevent snapping.
        """
        points = trajectory.joint_trajectory.points
        if not points:
            return
            
        self.get_logger().info(f'Retiming {len(points)} points for constant speed {speed} m/s...')
        
        new_points = []
        
        # Start with the first point
        points[0].time_from_start.sec = 0
        points[0].time_from_start.nanosec = 0
        points[0].velocities = []
        points[0].accelerations = []
        points[0].effort = []
        
        new_points.append(points[0])
        
        # Get FK for first point
        js = JointState()
        js.name = trajectory.joint_trajectory.joint_names
        js.position = points[0].positions
        prev_pose = self.get_fk(js)
        
        if not prev_pose:
            self.get_logger().error('Failed to compute FK for start point')
            return

        current_time = 0.0
        min_dt = 0.01 # Minimum time step (100Hz)
        max_joint_speed = 0.5 # rad/s limit to prevent snapping
        
        for i in range(1, len(points)):
            point = points[i]
            
            # Get FK for current point
            js.position = point.positions
            curr_pose = self.get_fk(js)
            
            if not curr_pose:
                continue
                
            # Calculate Cartesian distance
            dx = curr_pose.position.x - prev_pose.position.x
            dy = curr_pose.position.y - prev_pose.position.y
            dz = curr_pose.position.z - prev_pose.position.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Calculate required time for Cartesian speed
            dt_cart = dist / speed
            
            # Calculate required time for Joint speed
            prev_joints = new_points[-1].positions
            curr_joints = point.positions
            max_joint_diff = 0.0
            for j in range(len(curr_joints)):
                diff = abs(curr_joints[j] - prev_joints[j])
                if diff > max_joint_diff:
                    max_joint_diff = diff
            
            dt_joint = max_joint_diff / max_joint_speed
            
            # Use the largest required time
            dt = max(dt_cart, dt_joint)
            
            # If the point is too close (resulting in tiny dt), skip it to maintain smoothness
            # unless it's the last point
            if dt < min_dt and i < len(points) - 1:
                continue
                
            # If we kept it, update time
            current_time += dt
            
            point.time_from_start.sec = int(current_time)
            point.time_from_start.nanosec = int((current_time - int(current_time)) * 1e9)
            
            # Clear dynamics
            point.velocities = []
            point.accelerations = []
            point.effort = []
            
            new_points.append(point)
            prev_pose = curr_pose
            
        self.get_logger().info(f'Reduced points from {len(points)} to {len(new_points)}')
        self.get_logger().info(f'Total trajectory time: {current_time:.2f}s')
        
        trajectory.joint_trajectory.points = new_points

    def execute_path(self):
        if not self.waypoints:
            self.get_logger().error('No waypoints received')
            return

        # 0. Ensure quaternion continuity
        self.get_logger().info('Ensuring quaternion continuity...')
        all_waypoints = self.ensure_continuous_orientation(self.waypoints)
        total_points = len(all_waypoints)
        
        # Try to find a valid start configuration that allows the FULL path
        # We will try multiple IK seeds if the first one fails
        max_attempts = 30
        best_solution = None
        best_start_state = None
        
        self.get_logger().info(f'Attempting to find valid plan (max {max_attempts} attempts)...')
        
        for attempt in range(max_attempts):
            # Determine seed
            seed = None
            if attempt == 0:
                # First attempt: use current state
                seed = self.current_joint_state
            else:
                # Random seed to find alternative configurations (elbow up/down, etc)
                seed = JointState()
                if self.current_joint_state:
                    seed.name = self.current_joint_state.name
                else:
                    seed.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
                
                # Randomize positions within typical range
                seed.position = [random.uniform(-3.14, 3.14) for _ in range(len(seed.name))]

            # 1. Find IK
            start_robot_state = self.get_ik(all_waypoints[0], seed)
            if not start_robot_state:
                continue
                
            # 2. Plan Cartesian path from this start state
            request = GetCartesianPath.Request()
            request.header.frame_id = 'world'
            request.header.stamp = self.get_clock().now().to_msg()
            request.group_name = 'ur_manipulator'
            request.link_name = 'tool0'
            request.start_state = start_robot_state
            request.waypoints = all_waypoints
            request.max_step = 0.01
            request.jump_threshold = 0.0 
            request.avoid_collisions = True
            
            future = self.cartesian_path_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.done():
                response = future.result()
                if response.fraction >= 1.0:
                    self.get_logger().info(f'✓ Found valid plan on attempt {attempt+1}')
                    best_solution = response.solution
                    best_start_state = start_robot_state
                    break
                else:
                    # Only log failures occasionally to avoid spam
                    if attempt % 5 == 0:
                        self.get_logger().info(f'Attempt {attempt+1}: Only computed {response.fraction*100:.1f}%')

        if not best_solution:
            self.get_logger().error('Could not find a valid path configuration after all attempts.')
            return

        if best_solution.joint_trajectory.points:
            # 3. Move to the start configuration
            if not self.move_to_joint_state(best_start_state.joint_state):
                self.get_logger().error('Aborting: Could not reach start configuration')
                return
            
            # 4. Retime for constant speed
            self.retime_trajectory(best_solution, speed=0.05)
            
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = best_solution
            
            self.get_logger().info('Executing continuous path...')
            goal_future = self.execute_trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, goal_future)
            
            if goal_future.done() and goal_future.result().accepted:
                handle = goal_future.result()
                res_future = handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_future)
                
                if res_future.result().result.error_code.val == 1:
                    self.get_logger().info('✓ Execution complete')
                else:
                    self.get_logger().error('✗ Execution failed')
        else:
            self.get_logger().error('No trajectory generated')



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