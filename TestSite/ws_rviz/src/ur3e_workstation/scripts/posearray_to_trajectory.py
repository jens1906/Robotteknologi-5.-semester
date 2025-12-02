#!/usr/bin/env python3
"""
Surface follower that converts PoseArray into trajectory and executes it.
Uses MoveIt's compute_cartesian_path service and ExecuteTrajectory action.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, RobotTrajectory, MoveItErrorCodes
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient


class SurfaceFollower(Node):
    """Convert PoseArray to MoveIt trajectory using Cartesian path planning."""
    
    def __init__(self):
        super().__init__('posearray_to_trajectory')
        
        # Create service client for Cartesian path computation
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        
        self.get_logger().info('Waiting for /compute_cartesian_path service...')
        if not self.cartesian_path_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Cartesian path service not available!')
            raise RuntimeError('Service not available')
        
        self.get_logger().info('✓ Service ready')
        
        # Create action client for trajectory execution
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        
        self.get_logger().info('Waiting for /execute_trajectory action...')
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('ExecuteTrajectory action not available!')
            raise RuntimeError('Action not available')
        
        self.get_logger().info('✓ ExecuteTrajectory action ready')
        
        # Subscribe to PoseArray source
        self.subscription = self.create_subscription(
            PoseArray,
            '/tool_orientation/path',
            self.path_callback,
            10
        )
        
        self.get_logger().info('Waiting for PoseArray on /tool_orientation/path...')
    
    def path_callback(self, msg: PoseArray):
        """Callback when PoseArray is received - compute and execute trajectory."""
        self.get_logger().info(f"Received path with {len(msg.poses)} waypoints")
        
        if not msg.poses:
            self.get_logger().error('PoseArray is empty!')
            return
        
        # Downsample aggressively - MoveIt struggles with many waypoints
        waypoints = list(msg.poses)
        max_waypoints = 50  # Keep it low for speed
        
        if len(waypoints) > max_waypoints:
            # Always include first and last
            step = max(1, (len(waypoints) - 1) // (max_waypoints - 1))
            indices = list(range(0, len(waypoints), step))
            if indices[-1] != len(waypoints) - 1:
                indices.append(len(waypoints) - 1)  # Ensure last point included
            waypoints = [waypoints[i] for i in indices]
            self.get_logger().info(f"Downsampled from {len(msg.poses)} to {len(waypoints)} waypoints")
        
        # Compute Cartesian trajectory
        # Larger eef_step = MUCH faster computation
        fraction, trajectory = self.compute_cartesian_path(
            waypoints,
            eef_step=0.05,       # 50mm - large for speed (MoveIt will interpolate)
            jump_threshold=0.0   # Disable joint jump detection
        )
        
        self.get_logger().info(f"Path achieved {fraction * 100.0:.2f}% of poses")
        
        # Execute only if mostly complete
        if fraction > 0.95:
            self.get_logger().info('Path complete enough - executing...')
            self.execute_trajectory(trajectory)
        else:
            self.get_logger().warn("Path not fully planned, consider adjusting parameters")
    
    def compute_cartesian_path(self, waypoints, eef_step=0.01, jump_threshold=0.0,
                               group_name='ur_manipulator', frame_id='world', 
                               link_name='tool0'):
        """
        Compute Cartesian path through waypoints.
        
        Args:
            waypoints: List of Pose messages
            eef_step: Max step distance between trajectory points (meters)
            jump_threshold: Max joint angle jump (0.0 = no limit)
            group_name: MoveIt planning group name
            frame_id: Reference frame for poses
            link_name: End effector link name
        
        Returns:
            tuple: (fraction, trajectory) where fraction is 0.0-1.0
        """
        # Create service request
        request = GetCartesianPath.Request()
        
        # Set header
        request.header.frame_id = frame_id
        request.header.stamp = self.get_clock().now().to_msg()
        
        # Set planning group and link
        request.group_name = group_name
        request.link_name = link_name
        
        # Set waypoints
        request.waypoints = list(waypoints)
        
        # Set Cartesian path parameters
        request.max_step = eef_step
        request.jump_threshold = jump_threshold
        request.avoid_collisions = False  # Disable for speed - WARNING: ensure path is collision-free!
        
        # Empty constraints for maximum flexibility
        request.path_constraints = Constraints()
        
        # Use current robot state as start
        request.start_state.is_diff = True
        
        self.get_logger().info(f'Computing Cartesian path with {len(waypoints)} waypoints...')
        self.get_logger().info(f'Using eef_step={eef_step}m (larger = faster)')
        self.get_logger().warn('Collision checking DISABLED for speed!')
        
        # Call service with timeout
        future = self.cartesian_path_client.call_async(request)
        timeout = max(30.0, len(waypoints) * 0.3)  # 30s min, or 0.3s per waypoint
        self.get_logger().info(f'Waiting up to {timeout:.0f}s for computation...')
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if not future.done():
            self.get_logger().error(f'Service call timed out after {timeout:.0f}s!')
            self.get_logger().error(f'This path is too complex for Cartesian planning.')
            self.get_logger().error(f'Current: {len(waypoints)} waypoints, eef_step={eef_step}m')
            self.get_logger().error('Suggestions:')
            self.get_logger().error('  1. Reduce waypoints further (currently max 50)')
            self.get_logger().error('  2. Use joint-space planning instead')
            self.get_logger().error('  3. Split into smaller path segments')
            return (0.0, None)
        
        response = future.result()
        fraction = response.fraction
        
        if fraction > 0.0:
            num_points = len(response.solution.joint_trajectory.points)
            self.get_logger().info(f'✓ Trajectory has {num_points} points')
            return (fraction, response.solution)
        else:
            self.get_logger().error('Failed to compute path')
            return (fraction, None)
    
    def execute_trajectory(self, trajectory):
        """Execute the computed trajectory using ExecuteTrajectory action."""
        if trajectory is None:
            self.get_logger().error('Cannot execute - trajectory is None')
            return False
        
        # Create goal
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = trajectory
        
        self.get_logger().info('Sending trajectory to execute...')
        
        # Send goal
        exec_future = self.execute_trajectory_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future, timeout_sec=5.0)
        
        if not exec_future.done() or not exec_future.result().accepted:
            self.get_logger().error('✗ Trajectory execution goal rejected')
            return False
        
        exec_handle = exec_future.result()
        self.get_logger().info('✓ Trajectory accepted, executing...')
        
        # Wait for execution to complete
        exec_result_future = exec_handle.get_result_async()
        
        # Estimate timeout from trajectory duration
        if len(trajectory.joint_trajectory.points) > 0:
            last_point = trajectory.joint_trajectory.points[-1]
            duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
            timeout = duration + 10.0
        else:
            timeout = 30.0
        
        self.get_logger().info(f'Waiting for execution (timeout: {timeout:.1f}s)...')
        rclpy.spin_until_future_complete(self, exec_result_future, timeout_sec=timeout)
        
        if not exec_result_future.done():
            self.get_logger().error('✗ Execution timed out')
            return False
        
        exec_result = exec_result_future.result().result
        
        if exec_result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('✓ Trajectory executed successfully!')
            return True
        else:
            self.get_logger().error(f'✗ Execution failed with error code: {exec_result.error_code.val}')
            return False


def main(args=None):
    """Main function - runs the surface follower node."""
    print("\n" + "="*60)
    print("Surface Follower - PoseArray to Trajectory")
    print("="*60)
    print("\nThis node subscribes to /surface_path (PoseArray)")
    print("and automatically computes + executes trajectories.")
    print("\nMake sure:")
    print("  1. ROS 2 is running")
    print("  2. MoveIt is running (e.g., launch_ur_moveit.sh)")
    print("  3. Publish PoseArray to /surface_path topic")
    print("="*60 + "\n")
    
    rclpy.init(args=args)
    node = SurfaceFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
