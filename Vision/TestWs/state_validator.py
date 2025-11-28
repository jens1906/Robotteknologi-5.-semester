#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
import math

class StateValidator(Node):
    def __init__(self):
        super().__init__('state_validator')
        
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
            '/move_group/get_state_validity'
        )
        
        self.latest_joint_state = None
        self.get_logger().info("State validator node started")
    
    def joint_state_callback(self, msg):
        self.latest_joint_state = msg
        self.get_logger().info(f"Received joint state: {dict(zip(msg.name, msg.position))}")
        
        # Test state validity
        self.test_state_validity()
    
    def test_state_validity(self):
        if not self.latest_joint_state:
            return
            
        if not self.state_validity_client.service_is_ready():
            self.get_logger().warn("State validity service not ready")
            return
        
        # Create RobotState message
        robot_state = RobotState()
        robot_state.joint_state = self.latest_joint_state
        
        # Create service request
        request = GetStateValidity.Request()
        request.robot_state = robot_state
        request.group_name = "ur_manipulator"
        request.constraints = []
        
        # Call service
        future = self.state_validity_client.call_async(request)
        
        def handle_response(future):
            try:
                response = future.result()
                self.get_logger().info(f"State validity: {response.valid}")
                if not response.valid:
                    self.get_logger().error(f"Invalid state contacts: {response.contacts}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
        
        future.add_done_callback(handle_response)

def main(args=None):
    rclpy.init(args=args)
    node = StateValidator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()