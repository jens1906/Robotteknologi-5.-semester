#!/usr/bin/env python3
"""
Path Follower Node for UR3e Robot
Subscribes to a path topic containing rotation matrices and XYZ coordinates,
visualizes the path in RViz, and executes it using MoveIt.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float64MultiArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath
from std_srvs.srv import Trigger
import numpy as np
from scipy.spatial.transform import Rotation as R


class PathFollowerNode(Node):
    """
    Node that receives a path (rotation matrices + XYZ positions),
    visualizes it in RViz, and executes it using MoveIt Cartesian planning.
    """
    
    def __init__(self):
        super().__init__('path_follower_node')
        
        # Parameters
        self.declare_parameter('robot_name', 'ur3e_workstation')
        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('end_effector_link', 'tool0')
        self.declare_parameter('path_topic', '/scanning_path')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('cartesian_step', 0.01)  # 1cm steps
        self.declare_parameter('max_cartesian_speed', 0.1)  # m/s
        
        # Get parameters
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.cartesian_step = self.get_parameter('cartesian_step').value
        
        # Initialize MoveIt using action client
        self.get_logger().info('Initializing MoveIt interface...')
        try:
            # Create action client for MoveGroup
            self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
            
            # Create service client for Cartesian path planning
            self.cartesian_path_client = self.create_client(
                GetCartesianPath,
                '/compute_cartesian_path'
            )
            
            self.get_logger().info(f'Planning group: {self.planning_group}')
            self.get_logger().info(f'End effector link: {self.end_effector_link}')
            self.get_logger().info('Waiting for MoveIt services...')
            
            # Wait for services
            if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn('Cartesian path service not available yet')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MoveIt: {e}')
            return
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Float64MultiArray,
            self.get_parameter('path_topic').value,
            self.path_callback,
            10
        )
        
        # Publishers for visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/path_visualization', 10)
        self.path_pub = self.create_publisher(Marker, '/planned_path_line', 10)
        
        # Service for executing the path
        self.execute_service = self.create_service(
            Trigger,
            '/execute_cartesian_path',
            self.execute_path_service_callback
        )
        
        # Path storage
        self.waypoints = []
        self.path_received = False
        
        self.get_logger().info('Path Follower Node initialized!')
        self.get_logger().info(f'Waiting for path on topic: {self.get_parameter("path_topic").value}')
        self.get_logger().info('Expected format: [X1, Y1, Z1, R11, R12, R13, R21, R22, R23, R31, R32, R33, X2, Y2, Z2, ...]')
        self.get_logger().info('Where X,Y,Z are positions in meters and R is a 3x3 rotation matrix')
        self.get_logger().info('Service /execute_cartesian_path available for path execution')
    
    def execute_path_service_callback(self, request, response):
        """Service callback to execute the loaded path."""
        if not self.path_received or not self.waypoints:
            response.success = False
            response.message = 'No path loaded! Run import_line.sh first to load a path.'
            self.get_logger().error(response.message)
            return response
        
        self.get_logger().info('Executing path via service call...')
        success = self.execute_cartesian_path()
        
        response.success = success
        if success:
            response.message = f'Successfully computed Cartesian path with {len(self.waypoints)} waypoints'
        else:
            response.message = 'Failed to compute Cartesian path. Check logs for details.'
        
        return response
    
    def path_callback(self, msg):
        """
        Callback for receiving path data.
        Expected format: [X1, Y1, Z1, R11, R12, R13, R21, R22, R23, R31, R32, R33, X2, Y2, Z2, ...]
        Where X,Y,Z are positions in meters and R is a 3x3 rotation matrix.
        Data will be reshaped to (N,12) before use.
        """
        self.get_logger().info(f'Received path with {len(msg.data)} elements')
        
        # Parse the data
        data = np.array(msg.data)
        
        # Each waypoint has 12 values (3 for position + 9 for rotation matrix)
        if len(data) % 12 != 0:
            self.get_logger().error(f'Invalid path data length: {len(data)}. Must be multiple of 12.')
            return
        
        num_waypoints = len(data) // 12
        self.get_logger().info(f'Parsing {num_waypoints} waypoints...')
        
        # Reshape data to (N,12) as requested :)
        reshaped_data = data.reshape((num_waypoints, 12))
        self.get_logger().info(f'Data reshaped to ({num_waypoints}, 12)')
        
        # Parse waypoints
        self.waypoints = []
        for i in range(num_waypoints):
            # Extract position first (indices 0,1,2)
            position = reshaped_data[i, 0:3]
            
            # Extract rotation matrix (indices 3-11, then reshape to 3x3)
            rot_matrix = reshaped_data[i, 3:12].reshape((3, 3))
            
            # DEBUG: Log first waypoint raw data
            if i == 0:
                self.get_logger().info('='*60)
                self.get_logger().info('VISUALIZER DEBUG: First waypoint RAW data')
                self.get_logger().info('='*60)
                self.get_logger().info(f'Raw data[0:3] (position): {data[0:3]}')
                self.get_logger().info(f'Parsed position: [{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}]')
                self.get_logger().info(f'Rotation matrix determinant: {np.linalg.det(rot_matrix):.6f}')
                self.get_logger().info('='*60)
            
            # Convert rotation matrix to quaternion
            try:
                rotation = R.from_matrix(rot_matrix)
                quat = rotation.as_quat()  # Returns [x, y, z, w]
                
                # Create pose
                pose = Pose()
                pose.position.x = position[0]
                pose.position.y = position[1]
                pose.position.z = position[2]
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                
                self.waypoints.append(pose)
                
            except Exception as e:
                self.get_logger().warn(f'Failed to convert waypoint {i}: {e}')
                continue
        
        self.get_logger().info(f'Successfully parsed {len(self.waypoints)} waypoints')
        self.path_received = True
        
        # Visualize the path
        self.visualize_path()
        
        self.get_logger().info('Path visualization published to RViz!')
        self.get_logger().info('Call execute_path service to execute the path')
    
    def visualize_path(self):
        """Visualize the received path in RViz."""
        if not self.waypoints:
            return
        
        # Create marker array for waypoint poses
        marker_array = MarkerArray()
        
        # Create line strip for the path
        line_marker = Marker()
        line_marker.header.frame_id = self.reference_frame
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'path_line'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.005  # Line width
        line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
        line_marker.pose.orientation.w = 1.0
        
        # Add waypoint arrows and line points
        for i, pose in enumerate(self.waypoints):
            # Add point to line
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z
            line_marker.points.append(point)
            
            # Create arrow marker for pose orientation
            arrow = Marker()
            arrow.header.frame_id = self.reference_frame
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'waypoint_arrows'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose
            arrow.scale.x = 0.05  # Arrow length
            arrow.scale.y = 0.005  # Arrow width
            arrow.scale.z = 0.005  # Arrow height
            
            # Color gradient from red to blue
            ratio = i / max(len(self.waypoints) - 1, 1)
            arrow.color = ColorRGBA(r=1.0-ratio, g=0.0, b=ratio, a=0.7)
            
            marker_array.markers.append(arrow)
        
        # Publish markers
        self.marker_pub.publish(marker_array)
        self.path_pub.publish(line_marker)
        
        self.get_logger().info(f'Path visualization published: {len(self.waypoints)} waypoints')
    
    def execute_cartesian_path(self):
        """Execute the Cartesian path using MoveIt service."""
        if not self.waypoints:
            self.get_logger().error('No path to execute!')
            return False
        
        self.get_logger().info('='*60)
        self.get_logger().info('CARTESIAN PATH EXECUTION DEBUG INFO')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Number of waypoints: {len(self.waypoints)}')
        
        try:
            # Check if service is available
            if not self.cartesian_path_client.service_is_ready():
                self.get_logger().error('Cartesian path service is not available!')
                self.get_logger().error('Make sure MoveIt move_group node is running.')
                return False
            
            self.get_logger().info('✓ Cartesian path service is available')
            
            # Create request for Cartesian path
            request = GetCartesianPath.Request()
            request.header.frame_id = self.reference_frame
            request.header.stamp = self.get_clock().now().to_msg()
            request.group_name = self.planning_group
            request.link_name = self.end_effector_link
            
            self.get_logger().info(f'Planning group: {request.group_name}')
            self.get_logger().info(f'End effector link: {request.link_name}')
            self.get_logger().info(f'Reference frame: {request.header.frame_id}')
            
            # IMPORTANT: Leave start_state empty to use current robot state
            # This avoids the "start state doesn't match current state" error
            # The empty start_state will automatically use the current robot state
            request.start_state.is_diff = True  # Use diff mode (empty = current state)
            self.get_logger().info('Using current robot state as start state (is_diff=True)')
            
            # Convert poses to waypoints (must be Pose objects, not PoseStamped)
            self.get_logger().info(f'Converting {len(self.waypoints)} waypoints...')
            for i, pose in enumerate(self.waypoints):
                # Create a new Pose object to avoid reference issues
                waypoint_pose = Pose()
                waypoint_pose.position.x = float(pose.position.x)
                waypoint_pose.position.y = float(pose.position.y)
                waypoint_pose.position.z = float(pose.position.z)
                waypoint_pose.orientation.x = float(pose.orientation.x)
                waypoint_pose.orientation.y = float(pose.orientation.y)
                waypoint_pose.orientation.z = float(pose.orientation.z)
                waypoint_pose.orientation.w = float(pose.orientation.w)
                request.waypoints.append(waypoint_pose)
                
                # Log ALL waypoints for debugging
                self.get_logger().info(
                    f'  Waypoint {i}: pos=[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}] '
                    f'quat=[{pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f}]'
                )
            
            request.max_step = float(self.cartesian_step)
            request.jump_threshold = 0.0
            request.avoid_collisions = True
            
            self.get_logger().info(f'Max step size: {request.max_step}m')
            self.get_logger().info(f'Jump threshold: {request.jump_threshold}')
            self.get_logger().info(f'Collision avoidance: {request.avoid_collisions}')
            
            # WORKAROUND: Add a small delay to ensure robot state is updated
            # This fixes the "start state doesn't match current state" bug
            self.get_logger().info('Waiting 0.5s for robot state to update...')
            import time
            time.sleep(0.5)
            
            # Call service
            self.get_logger().info('-'*60)
            self.get_logger().info('Calling Cartesian path service...')
            
            future = self.cartesian_path_client.call_async(request)
            
            # Wait for result with timeout
            timeout_sec = 10.0
            self.get_logger().info(f'Waiting for response (timeout: {timeout_sec}s)...')
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            
            if not future.done():
                self.get_logger().error('✗ Service call timed out!')
                self.get_logger().error('MoveIt is not responding. Check if move_group is running.')
                return False
            
            self.get_logger().info('✓ Service call completed')
            
            if future.result() is not None:
                response = future.result()
                fraction = response.fraction
                error_code = response.error_code.val
                
                self.get_logger().info('='*60)
                self.get_logger().info('CARTESIAN PATH RESULT')
                self.get_logger().info('='*60)
                self.get_logger().info(f'Success fraction: {fraction*100:.1f}%')
                self.get_logger().info(f'Error code: {error_code}')
                
                # Detailed error code explanation
                error_code_names = {
                    1: 'SUCCESS',
                    -1: 'FAILURE',
                    -2: 'PLANNING_FAILED',
                    -3: 'INVALID_MOTION_PLAN',
                    -4: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
                    -5: 'CONTROL_FAILED',
                    -6: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
                    -7: 'TIMED_OUT',
                    -10: 'PREEMPTED',
                    -11: 'START_STATE_IN_COLLISION',
                    -12: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
                    -13: 'GOAL_IN_COLLISION',
                    -14: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
                    -15: 'GOAL_CONSTRAINTS_VIOLATED',
                    -16: 'INVALID_GROUP_NAME',
                    -17: 'INVALID_GOAL_CONSTRAINTS',
                    -18: 'INVALID_ROBOT_STATE',
                    -19: 'INVALID_LINK_NAME',
                    -20: 'INVALID_OBJECT_NAME',
                    -21: 'FRAME_TRANSFORM_FAILURE',
                    -22: 'COLLISION_CHECKING_UNAVAILABLE',
                    -23: 'ROBOT_STATE_STALE',
                    -24: 'SENSOR_INFO_STALE',
                    -31: 'NO_IK_SOLUTION',
                }
                error_name = error_code_names.get(error_code, f'UNKNOWN_ERROR_{error_code}')
                self.get_logger().info(f'Error code meaning: {error_name}')
                
                if len(response.solution.joint_trajectory.points) > 0:
                    self.get_logger().info(f'Trajectory points computed: {len(response.solution.joint_trajectory.points)}')
                else:
                    self.get_logger().warn('No trajectory points generated!')
                
                if fraction < 0.9:
                    self.get_logger().warn('-'*60)
                    self.get_logger().warn(f'Only {fraction*100:.1f}% of path could be planned!')
                    self.get_logger().warn('POSSIBLE REASONS:')
                    if error_code == -31:
                        self.get_logger().warn('  - NO_IK_SOLUTION: Waypoints are unreachable')
                        self.get_logger().warn('  - Try adjusting waypoint positions closer to robot')
                        self.get_logger().warn('  - Try simplifying waypoint orientations')
                    elif error_code == -11:
                        self.get_logger().warn('  - START_STATE_IN_COLLISION')
                        self.get_logger().warn('  - Move robot to a collision-free position first')
                    elif error_code == -13:
                        self.get_logger().warn('  - GOAL_IN_COLLISION')
                        self.get_logger().warn('  - Path waypoints collide with environment')
                    else:
                        self.get_logger().warn('  - Waypoints may be out of robot reach')
                        self.get_logger().warn('  - Path may cause collisions')
                        self.get_logger().warn('  - Orientations may not be achievable')
                    self.get_logger().warn('-'*60)
                
                if fraction > 0.0:
                    self.get_logger().info('='*60)
                    self.get_logger().info('✓ SUCCESS: Path computed!')
                    self.get_logger().info('='*60)
                    self.get_logger().info('Note: This only computed the path, did not execute it')
                    self.get_logger().info('To execute:')
                    self.get_logger().info('  1. Use MoveIt RViz panel to execute')
                    self.get_logger().info('  2. Or implement execution in this node')
                    return True
                else:
                    self.get_logger().error('='*60)
                    self.get_logger().error('✗ FAILED: Could not compute any part of the path')
                    self.get_logger().error('='*60)
                    self.get_logger().error(f'Error: {error_name} (code {error_code})')
                    self.get_logger().error('DEBUGGING STEPS:')
                    self.get_logger().error('  1. Check if waypoints are visible in RViz')
                    self.get_logger().error('  2. Verify waypoints are within robot reach (green = reachable)')
                    self.get_logger().error('  3. Try moving robot closer to first waypoint manually')
                    self.get_logger().error('  4. Simplify the path (fewer waypoints, simpler orientations)')
                    self.get_logger().error('  5. Check Terminal 1 (launch_ur_moveit.sh) for MoveIt errors')
                    return False
            else:
                self.get_logger().error('✗ Service returned None!')
                return False
            
        except Exception as e:
            self.get_logger().error('='*60)
            self.get_logger().error('EXCEPTION OCCURRED')
            self.get_logger().error('='*60)
            self.get_logger().error(f'Error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
    def execute_point_to_point(self):
        """
        Execute path by moving to each waypoint sequentially.
        Note: This requires MoveGroup action to be properly configured.
        """
        self.get_logger().warn('Point-to-point execution not fully implemented yet.')
        self.get_logger().warn('Use the Cartesian path method or MoveIt RViz panel for execution.')
        return False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PathFollowerNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
