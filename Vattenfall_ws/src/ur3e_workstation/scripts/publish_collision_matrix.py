#!/usr/bin/env python3
"""
Publishes allowed collision matrix to disable collision checking between 
workstation parts that are meant to be in contact.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, AllowedCollisionMatrix, AllowedCollisionEntry

class CollisionMatrixPublisher(Node):
    def __init__(self):
        super().__init__('collision_matrix_publisher')
        
        # Wait for the planning scene service
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /apply_planning_scene service...')
        
        self.publish_collision_matrix()
    
    def publish_collision_matrix(self):
        # Create planning scene message
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        
        # Create allowed collision matrix
        acm = AllowedCollisionMatrix()
        
        # Define all links
        links = [
            'world', 'bordplade', 'mount', 'testpladekurve',
            'base_link', 'base_link_inertia', 'shoulder_link', 'upper_arm_link',
            'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link',
            'tool0', 'realsense_holder', 'rsd435'
        ]
        
        acm.entry_names = links
        
        # Pairs that should NOT check for collisions
        disabled_pairs = [
            ('world', 'bordplade'),
            ('bordplade', 'mount'),
            ('mount', 'base_link'),
            ('mount', 'base_link_inertia'),
            ('mount', 'shoulder_link'),
            ('bordplade', 'base_link'),
            ('bordplade', 'base_link_inertia'),
            ('bordplade', 'shoulder_link'),
            ('bordplade', 'upper_arm_link'),
            ('testpladekurve', 'base_link'),
            ('testpladekurve', 'base_link_inertia'),
            ('testpladekurve', 'shoulder_link'),
            ('tool0', 'realsense_holder'),
            ('realsense_holder', 'rsd435'),
            ('wrist_3_link', 'realsense_holder'),
            ('wrist_3_link', 'rsd435'),
            ('wrist_2_link', 'realsense_holder'),
            ('base_link', 'base_link_inertia'),
            ('base_link_inertia', 'shoulder_link'),
            ('shoulder_link', 'upper_arm_link'),
            ('upper_arm_link', 'forearm_link'),
            ('forearm_link', 'wrist_1_link'),
            ('wrist_1_link', 'wrist_2_link'),
            ('wrist_2_link', 'wrist_3_link'),
            ('wrist_3_link', 'tool0'),
            # Add pairs that were causing issues
            ('bordplade', 'forearm_link'),
            ('bordplade', 'wrist_1_link'),
        ]
        
        # Build the collision matrix
        for link in links:
            entry = AllowedCollisionEntry()
            entry.enabled = []
            for other_link in links:
                # Check if this pair should have collisions disabled
                should_disable = (
                    (link, other_link) in disabled_pairs or 
                    (other_link, link) in disabled_pairs or
                    link == other_link
                )
                
                # Explicitly log the status for the camera and test plate
                if (link == 'testpladekurve' and other_link == 'rsd435') or \
                   (link == 'rsd435' and other_link == 'testpladekurve'):
                     self.get_logger().info(f'Collision check for {link} <-> {other_link}: {"DISABLED" if should_disable else "ENABLED"}')

                entry.enabled.append(should_disable)
            acm.entry_values.append(entry)
        
        planning_scene.allowed_collision_matrix = acm
        
        # Apply the planning scene
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('Successfully published collision matrix!')
        else:
            self.get_logger().error('Failed to publish collision matrix')

def main(args=None):
    rclpy.init(args=args)
    node = CollisionMatrixPublisher()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
