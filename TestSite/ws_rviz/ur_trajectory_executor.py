#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import socket
import threading
import time
from builtin_interfaces.msg import Duration

class URTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('ur_trajectory_executor')
        
        # Parameters
        self.declare_parameter('robot_ip', '192.168.0.100')
        self.declare_parameter('command_port', 30001)
        
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.command_port = self.get_parameter('command_port').get_parameter_value().integer_value
        
        # Joint names for UR3e
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Action servers for the controllers MoveIt expects
        self.scaled_trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            'scaled_joint_trajectory_controller/follow_joint_trajectory',
            self.execute_scaled_trajectory
        )
        
        self.trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory',
            self.execute_trajectory
        )
        
        self.get_logger().info(f'UR Trajectory Executor started for robot at {self.robot_ip}')
        self.get_logger().info('Action servers available:')
        self.get_logger().info('  - scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.get_logger().info('  - joint_trajectory_controller/follow_joint_trajectory')

    def connect_robot_command_interface(self):
        """Connect to robot command interface"""
        try:
            command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            command_socket.settimeout(10.0)
            command_socket.connect((self.robot_ip, self.command_port))
            self.get_logger().info('Connected to robot command interface')
            return command_socket
        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot: {e}')
            return None

    def trajectory_to_urscript_basic(self, trajectory):
        """Convert ROS trajectory to URScript - improved version that actually works"""
        if not trajectory.points:
            return None
        
        self.get_logger().info(f'Converting trajectory with {len(trajectory.points)} points')
        
        # Use fewer points to make it more reliable
        step = max(1, len(trajectory.points) // 5)  # Take about 5 points max
        selected_points = []
        
        for i in range(0, len(trajectory.points), step):
            selected_points.append(trajectory.points[i])
        
        # Always include the final point
        if len(selected_points) > 1 and selected_points[-1] != trajectory.points[-1]:
            selected_points.append(trajectory.points[-1])
        
        self.get_logger().info(f'Using {len(selected_points)} waypoints')
        
        # Generate URScript with proper formatting
        script_lines = []
        
        # Add initial message
        script_lines.append('textmsg("Starting trajectory execution")')
        
        for i, point in enumerate(selected_points):
            if len(point.positions) != 6:
                continue
                
            positions = [round(pos, 4) for pos in point.positions]
            
            # Use movej with conservative parameters
            # a=0.5 (acceleration), v=0.3 (velocity), t=0 (time), r=0.02 (blend radius)
            if i == len(selected_points) - 1:
                # Final point - no blend radius
                script_lines.append(f'movej({positions}, a=0.5, v=0.3, t=0, r=0)')
            else:
                # Intermediate points - small blend radius for smooth motion
                script_lines.append(f'movej({positions}, a=0.5, v=0.3, t=0, r=0.02)')
            
            if i < 3 or i == len(selected_points) - 1:
                self.get_logger().info(f'Waypoint {i}: {positions}')
        
        # Add completion message
        script_lines.append('textmsg("Trajectory execution completed")')
        
        # Join with proper newlines and add final newline
        urscript = '\n'.join(script_lines) + '\n'
        
        self.get_logger().info("Generated URScript:")
        for line in script_lines[:3]:  # Show first few lines
            self.get_logger().info(f"  {line}")
        self.get_logger().info(f"  ... ({len(script_lines)} total lines)")
        
        return urscript

    def execute_scaled_trajectory(self, goal_handle):
        """Execute trajectory on scaled joint trajectory controller"""
        return self._execute_trajectory_common(goal_handle, "scaled_joint_trajectory_controller")

    def execute_trajectory(self, goal_handle):
        """Execute trajectory on regular joint trajectory controller"""
        return self._execute_trajectory_common(goal_handle, "joint_trajectory_controller")

    def _execute_trajectory_common(self, goal_handle, controller_name):
        """Common trajectory execution logic"""
        self.get_logger().info(f'Executing trajectory on {controller_name}')
        
        trajectory = goal_handle.request.trajectory
        
        # Basic validation
        if not trajectory.points:
            self.get_logger().error('Trajectory has no points')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result
        
        # Convert to URScript
        urscript = self.trajectory_to_urscript_basic(trajectory)
        if not urscript:
            self.get_logger().error('Failed to convert trajectory to URScript')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result
        
        # Accept the goal
        goal_handle.succeed()
        
        # Send to robot
        command_socket = self.connect_robot_command_interface()
        if not command_socket:
            self.get_logger().error('Failed to connect to robot')
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            return result
        
        try:
            self.get_logger().info('Sending trajectory to robot')
            
            # Send URScript with proper encoding and termination
            urscript_bytes = urscript.encode('utf-8')
            total_sent = 0
            
            while total_sent < len(urscript_bytes):
                sent = command_socket.send(urscript_bytes[total_sent:])
                if sent == 0:
                    raise RuntimeError("Socket connection broken")
                total_sent += sent
            
            self.get_logger().info(f'Sent {total_sent} bytes to robot')
            
            # Give robot time to parse and start execution
            time.sleep(1.0)
            
            # Try to read response to confirm receipt
            try:
                command_socket.settimeout(2.0)
                response = command_socket.recv(1024)
                if response:
                    self.get_logger().info(f'Robot response: {response.decode("utf-8", errors="ignore")}')
            except socket.timeout:
                self.get_logger().info('No immediate response from robot (this is normal)')
            except Exception as e:
                self.get_logger().warn(f'Could not read robot response: {e}')
            
            # Estimate execution time based on trajectory
            num_waypoints = len([p for p in trajectory.points if len(p.positions) == 6])
            # Conservative estimate: 2-3 seconds per waypoint minimum
            execution_time = max(8.0, num_waypoints * 2.5)
            
            self.get_logger().info(f'Waiting {execution_time:.1f}s for trajectory execution')
            
            # Wait for execution to complete
            time.sleep(execution_time)
            
            self.get_logger().info('Trajectory execution should be completed')
            
        except Exception as e:
            self.get_logger().error(f'Error during trajectory execution: {e}')
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            return result
        finally:
            try:
                command_socket.close()
            except:
                pass
        
        # Return success
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

def main(args=None):
    rclpy.init(args=args)
    
    try:
        executor_node = URTrajectoryExecutor()
        rclpy.spin(executor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'executor_node' in locals():
            executor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()