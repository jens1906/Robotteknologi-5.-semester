#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState, Constraints
import math

class DeepStateValidator(Node):
    def __init__(self):
        super().__init__('deep_state_validator')
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        
        # Create service client for state validation
        self.state_validity_client = self.create_client(
            GetStateValidity, 
            '/check_state_validity'
        )
        
        self.latest_joint_state = None
        self.test_count = 0
        self.get_logger().info("Deep state validator node started")
    
    def joint_state_callback(self, msg):
        self.latest_joint_state = msg
        
        # Only test every 100th message to avoid spam
        self.test_count += 1
        if self.test_count % 100 == 0:
            self.get_logger().info(f"Received joint state: {dict(zip(msg.name, msg.position))}")
            self.test_state_validity_detailed()
    
    def test_state_validity_detailed(self):
        if not self.latest_joint_state:
            return
            
        if not self.state_validity_client.service_is_ready():
            self.get_logger().warn("State validity service not ready")
            return
        
        # Create RobotState message with current joint state
        robot_state = RobotState()
        robot_state.joint_state = self.latest_joint_state
        
        # Create service request with detailed validation
        request = GetStateValidity.Request()
        request.robot_state = robot_state
        request.group_name = "ur_manipulator" 
        request.constraints = Constraints()  # Empty constraints
        
        # Call service
        future = self.state_validity_client.call_async(request)
        
        def handle_response(future):
            try:
                response = future.result()
                self.get_logger().info(f"=== STATE VALIDATION RESULT ===")
                self.get_logger().info(f"Valid: {response.valid}")
                self.get_logger().info(f"Contacts: {response.contacts}")
                self.get_logger().info(f"Cost sources: {response.cost_sources}")
                self.get_logger().info(f"Constraint result: {response.constraint_result}")
                
                # Log joint values for analysis
                joint_dict = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
                self.get_logger().info(f"Joint positions (rad): {joint_dict}")
                
                # Convert to degrees for easier checking
                joint_dict_deg = {name: math.degrees(pos) for name, pos in joint_dict.items()}
                self.get_logger().info(f"Joint positions (deg): {joint_dict_deg}")
                
                # Check if any joints are near limits
                ur3e_limits = {
                    'shoulder_pan_joint': (-360, 360),
                    'shoulder_lift_joint': (-360, 360), 
                    'elbow_joint': (-180, 180),
                    'wrist_1_joint': (-360, 360),
                    'wrist_2_joint': (-360, 360),
                    'wrist_3_joint': (-360, 360)
                }
                
                for joint_name, (min_deg, max_deg) in ur3e_limits.items():
                    if joint_name in joint_dict_deg:
                        pos_deg = joint_dict_deg[joint_name]
                        if pos_deg < min_deg or pos_deg > max_deg:
                            self.get_logger().error(f"Joint {joint_name} OUT OF BOUNDS: {pos_deg:.1f}° (limits: {min_deg}° to {max_deg}°)")
                        else:
                            self.get_logger().info(f"Joint {joint_name} OK: {pos_deg:.1f}° (limits: {min_deg}° to {max_deg}°)")
                
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
        
        future.add_done_callback(handle_response)

def main(args=None):
    rclpy.init(args=args)
    node = DeepStateValidator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()