#!/usr/bin/env python3
"""
Script to monitor and normalize continuous joint angles.
Prevents 'Start state out of bounds' errors by wrapping joint angles.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math


class JointAngleNormalizer(Node):
    def __init__(self):
        super().__init__('joint_angle_normalizer')
        
        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher to correct joint positions
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        
        self.last_positions = None
        self.joint_names = None
        
        self.get_logger().info('Joint Angle Normalizer started')
        self.get_logger().info('Monitoring for out-of-bounds continuous joints...')

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def joint_state_callback(self, msg):
        if self.joint_names is None:
            self.joint_names = msg.name
        
        # Check for out-of-bounds angles
        needs_correction = False
        corrected_positions = list(msg.position)
        
        for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
            # Focus on continuous joints (wrist joints typically)
            if 'wrist' in name or 'shoulder_pan' in name:
                if abs(pos) > 2 * math.pi:  # More than one full rotation
                    normalized = self.normalize_angle(pos)
                    self.get_logger().warn(
                        f'{name}: {pos:.3f} rad is out of normal range! '
                        f'Normalizing to {normalized:.3f} rad'
                    )
                    corrected_positions[i] = normalized
                    needs_correction = True
        
        # Only publish correction if needed
        if needs_correction:
            self.get_logger().info('Publishing normalized joint positions...')
            correction_msg = Float64MultiArray()
            correction_msg.data = corrected_positions
            self.publisher.publish(correction_msg)
        
        self.last_positions = msg.position


def main(args=None):
    rclpy.init(args=args)
    normalizer = JointAngleNormalizer()
    
    try:
        rclpy.spin(normalizer)
    except KeyboardInterrupt:
        pass
    finally:
        normalizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
