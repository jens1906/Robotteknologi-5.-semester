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

    def trajectory_to_urscript_servoj(self, trajectory):
        """Convert ROS trajectory to URScript using servoj for precise following"""
        if not trajectory.points:
            return None
        
        self.get_logger().info(f'Converting trajectory with {len(trajectory.points)} points using servoj')
        
        # Calculate time between points
        script_lines = []
        script_lines.append('# MoveIt Trajectory Execution using servoj')
        script_lines.append('def execute_moveit_trajectory():')
        
        # Get all valid points
        valid_points = [p for p in trajectory.points if len(p.positions) == 6]
        
        if len(valid_points) < 2:
            self.get_logger().warn('Trajectory has less than 2 valid points, using simple movej')
            return self.trajectory_to_urscript_movej_simple(trajectory)
        
        # Calculate timing
        total_time = 0.0
        if valid_points[-1].time_from_start.sec > 0 or valid_points[-1].time_from_start.nanosec > 0:
            total_time = valid_points[-1].time_from_start.sec + valid_points[-1].time_from_start.nanosec / 1e9
        else:
            # Estimate timing if not provided
            total_time = len(valid_points) * 0.1  # 100ms per point
        
        self.get_logger().info(f'Total trajectory time: {total_time:.2f}s')
        
        # Start servo mode
        script_lines.append('  textmsg("Starting precise trajectory following with servoj")')
        
        # Move to first position safely
        first_pos = [round(pos, 4) for pos in valid_points[0].positions]
        script_lines.append(f'  movej({first_pos}, a=0.5, v=0.3)')
        
        # Enter servoj mode for precise following
        script_lines.append('  # Enter servo mode for precise trajectory following')
        
        # Use servoj for each point
        for i, point in enumerate(valid_points):
            positions = [round(pos, 4) for pos in point.positions]
            
            # Calculate time for this point
            if point.time_from_start.sec > 0 or point.time_from_start.nanosec > 0:
                point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            else:
                point_time = i * 0.1  # Default 100ms per point
            
            # Use servoj with lookahead_time and gain for smooth motion
            # servoj(q, a, v, t, lookahead_time, gain)
            script_lines.append(f'  servoj({positions}, a=0.5, v=0.5, t=0.08, lookahead_time=0.1, gain=300)')
            
            if i < 3 or i == len(valid_points) - 1:
                self.get_logger().info(f'Servoj point {i}: {positions}')
        
        # Add a small wait to ensure completion
        script_lines.append('  sleep(0.5)')
        script_lines.append('  textmsg("Trajectory execution completed")')
        script_lines.append('end')
        script_lines.append('')
        script_lines.append('# Execute the trajectory')
        script_lines.append('execute_moveit_trajectory()')
        
        urscript = '\n'.join(script_lines)
        
        self.get_logger().info("Generated servoj URScript (first few lines):")
        for line in script_lines[:8]:
            self.get_logger().info(f"  {line}")
        
        return urscript

    def trajectory_to_urscript_movej_simple(self, trajectory):
        """Fallback: Convert ROS trajectory to URScript using movej with blend radius"""
        if not trajectory.points:
            return None
        
        self.get_logger().info(f'Converting trajectory with {len(trajectory.points)} points using movej with blending')
        
        script_lines = []
        script_lines.append('# MoveIt Trajectory Execution using movej with blending')
        script_lines.append('def execute_moveit_trajectory():')
        script_lines.append('  textmsg("Starting trajectory execution with blended movej")')
        
        valid_points = [p for p in trajectory.points if len(p.positions) == 6]
        
        for i, point in enumerate(valid_points[::max(1, len(valid_points)//8)]):  # Use max 8 points
            positions = [round(pos, 4) for pos in point.positions]
            
            # Use blend radius for smooth motion, except for the last point
            if i == len(valid_points) - 1:
                script_lines.append(f'  movej({positions}, a=0.3, v=0.5, r=0)')  # No blend on final point
            else:
                script_lines.append(f'  movej({positions}, a=0.3, v=0.5, r=0.05)')  # Small blend radius
        
        script_lines.append('  textmsg("Trajectory execution completed")')
        script_lines.append('end')
        script_lines.append('')
        script_lines.append('# Execute the trajectory')
        script_lines.append('execute_moveit_trajectory()')
        
        return '\n'.join(script_lines)

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
        
        # Convert to URScript using servoj for precise following
        urscript = self.trajectory_to_urscript_servoj(trajectory)
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
            self.get_logger().info('Sending servoj trajectory to robot...')
            
            # Send URScript
            urscript_bytes = urscript.encode('utf-8')
            command_socket.send(urscript_bytes)
            
            self.get_logger().info(f'Sent {len(urscript_bytes)} bytes of servoj script to robot')
            
            # Calculate execution time
            valid_points = [p for p in trajectory.points if len(p.positions) == 6]
            
            # Base execution time on number of points and servoj timing
            execution_time = len(valid_points) * 0.08 + 2.0  # 80ms per servoj point + overhead
            execution_time = max(execution_time, 3.0)  # Minimum 3 seconds
            
            self.get_logger().info(f'Waiting {execution_time:.1f}s for servoj trajectory execution...')
            
            # Wait for execution
            time.sleep(execution_time)
            
            self.get_logger().info('Trajectory execution completed')
            
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