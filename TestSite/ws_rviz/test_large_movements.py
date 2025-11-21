#!/usr/bin/env python3
"""
Test script to create large, visible movements through MoveIt
This will help verify that the robot actually moves visibly
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayRobotState, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import time
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions, 
    Constraints,
    WorkspaceParameters
)

class MoveItTester(Node):
    def __init__(self):
        super().__init__('moveit_tester')
        
        # MoveGroup action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Wait for MoveIt to be ready
        self.get_logger().info("Waiting for MoveIt action server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("MoveIt action server ready!")
        
    def create_joint_goal(self, joint_values, joint_names=None):
        """Create a motion plan request for joint space goal"""
        if joint_names is None:
            joint_names = [
                'shoulder_pan_joint',
                'shoulder_lift_joint', 
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ]
        
        # Create the motion plan request
        req = MoveGroup.Goal()
        req.request = MotionPlanRequest()
        req.request.group_name = 'ur_manipulator'
        req.request.num_planning_attempts = 10
        req.request.allowed_planning_time = 5.0
        req.request.planner_id = ''  # Use default
        
        # Set joint constraints
        req.request.goal_constraints = [Constraints()]
        
        # Add joint constraints
        from moveit_msgs.msg import JointConstraint
        
        for i, (name, value) in enumerate(zip(joint_names, joint_values)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            
            req.request.goal_constraints[0].joint_constraints.append(joint_constraint)
        
        # Set planning options
        req.planning_options = PlanningOptions()
        req.planning_options.plan_only = False  # Plan AND execute
        req.planning_options.look_around = False
        req.planning_options.look_around_attempts = 0
        req.planning_options.max_safe_execution_cost = 1.0
        req.planning_options.replan = True
        req.planning_options.replan_attempts = 5
        req.planning_options.replan_delay = 2.0
        
        return req
    
    def move_to_position(self, joint_values, description=""):
        """Move robot to specified joint positions"""
        self.get_logger().info(f"Moving to: {description}")
        self.get_logger().info(f"Joint values: {joint_values}")
        
        # Create goal
        goal = self.create_joint_goal(joint_values)
        
        # Send goal
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False
            
        self.get_logger().info("Goal accepted, waiting for result...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("✓ Movement completed successfully!")
            return True
        else:
            self.get_logger().error(f"Movement failed with error code: {result.error_code.val}")
            return False
    
    def run_movement_test(self):
        """Run a series of large, visible movements"""
        self.get_logger().info("=== Starting Visible Movement Test ===")
        
        # Define dramatic test positions (in radians)
        positions = [
            {
                'name': 'Home Position', 
                'joints': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            },
            {
                'name': 'Right Extended',
                'joints': [1.57, -1.2, 0.8, -1.0, -1.57, 0.0]  # 90° shoulder pan
            },
            {
                'name': 'Left Extended', 
                'joints': [-1.57, -1.2, 0.8, -1.0, -1.57, 0.0]  # -90° shoulder pan
            },
            {
                'name': 'Up Position',
                'joints': [0.0, -0.5, -0.8, -1.2, -1.57, 0.0]  # Arm up
            },
            {
                'name': 'Forward Reach',
                'joints': [0.0, -2.0, 1.5, -1.0, -1.57, 0.0]  # Reach forward
            },
            {
                'name': 'Back to Home',
                'joints': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            }
        ]
        
        for i, pos in enumerate(positions):
            self.get_logger().info(f"\n--- Movement {i+1}/{len(positions)}: {pos['name']} ---")
            
            success = self.move_to_position(pos['joints'], pos['name'])
            
            if success:
                self.get_logger().info(f"✓ Successfully moved to {pos['name']}")
                self.get_logger().info("Waiting 3 seconds before next movement...")
                time.sleep(3)
            else:
                self.get_logger().error(f"✗ Failed to move to {pos['name']}")
                self.get_logger().info("Continuing with next movement...")
                time.sleep(1)
        
        self.get_logger().info("\n=== Movement Test Complete ===")

def main():
    rclpy.init()
    
    tester = MoveItTester()
    
    try:
        # Wait a moment for everything to initialize
        time.sleep(2)
        
        # Run the test
        tester.run_movement_test()
        
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
    except Exception as e:
        tester.get_logger().error(f"Test failed: {e}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()