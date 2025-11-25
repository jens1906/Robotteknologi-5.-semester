#!/usr/bin/env python3
"""
RoboDK to ROS2 MoveIt Bridge

This script allows you to control the real UR3e robot from RoboDK 
with full collision checking through MoveIt.

Usage from RoboDK Python API:
    from robodk_moveit_bridge import RoboDKMoveItBridge
    
    bridge = RoboDKMoveItBridge()
    
    # Get joint positions from RoboDK robot
    robodk_joints = robot.Joints()  # [j1, j2, j3, j4, j5, j6] in degrees
    
    # Send to real robot with collision checking
    success = bridge.move_to_position(robodk_joints)
    
    if not success:
        print("Movement blocked - would cause collision!")
"""

import rclpy
from collision_aware_mover import CollisionAwareMover
import math
import time


class RoboDKMoveItBridge:
    """
    Bridge between RoboDK and ROS2 MoveIt for collision-aware control.
    """
    
    def __init__(self):
        """Initialize the bridge and connect to MoveIt"""
        if not rclpy.ok():
            rclpy.init()
            
        self.mover = CollisionAwareMover()
        self._initialized = True
        print("✅ RoboDK-MoveIt Bridge initialized")
        print("   All movements will be collision-checked!")
        
    def move_to_position(self, joint_positions_deg, velocity_scaling=0.1):
        """
        Move robot to specified joint positions (RoboDK format: degrees).
        
        Args:
            joint_positions_deg: List of 6 joint angles in DEGREES [j1, j2, j3, j4, j5, j6]
            velocity_scaling: Speed factor (0.0-1.0), default 0.1 for safety
            
        Returns:
            True if movement succeeded, False if collision detected or planning failed
        """
        if not self._initialized:
            print("❌ Bridge not initialized!")
            return False
            
        # Convert degrees to radians
        joint_positions_rad = [math.radians(angle) for angle in joint_positions_deg]
        
        print(f"\n🤖 Moving to position:")
        print(f"   Degrees: {[f'{p:.1f}°' for p in joint_positions_deg]}")
        print(f"   Radians: {[f'{p:.3f}' for p in joint_positions_rad]}")
        print(f"   Checking collisions with:")
        print(f"   - Robot self-collision")
        print(f"   - Testsetup/testplate")
        print(f"   - RealSense camera")
        
        # Use MoveIt for collision-aware movement
        success = self.mover.move_to_joint_positions(
            joint_positions_rad,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=velocity_scaling
        )
        
        if success:
            print("✅ Movement executed safely\n")
        else:
            print("❌ Movement BLOCKED - would cause collision!\n")
            
        return success
        
    def move_to_home(self):
        """Move to home position with collision checking"""
        home_position_deg = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
        return self.move_to_position(home_position_deg)
        
    def get_current_position(self):
        """
        Get current robot joint positions.
        
        Returns:
            List of 6 joint angles in DEGREES, or None if not available
        """
        if self.mover._current_joint_state is None:
            print("⚠️ No joint state available yet")
            return None
            
        # Extract positions for UR joints in correct order
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        positions_rad = []
        for name in joint_names:
            try:
                idx = self.mover._current_joint_state.name.index(name)
                positions_rad.append(self.mover._current_joint_state.position[idx])
            except (ValueError, IndexError):
                print(f"⚠️ Joint {name} not found in joint state")
                return None
                
        # Convert to degrees
        positions_deg = [math.degrees(angle) for angle in positions_rad]
        return positions_deg
        
    def check_trajectory(self, trajectory_points_deg, velocity_scaling=0.1):
        """
        Check if a trajectory is collision-free by testing each waypoint.
        
        Args:
            trajectory_points_deg: List of waypoints, each is [j1, j2, j3, j4, j5, j6] in degrees
            velocity_scaling: Speed factor for execution
            
        Returns:
            (success, failed_waypoint_index) 
            - If all waypoints are valid: (True, None)
            - If collision detected: (False, index_of_problem_waypoint)
        """
        print(f"\n🔍 Checking trajectory with {len(trajectory_points_deg)} waypoints...")
        
        for i, waypoint_deg in enumerate(trajectory_points_deg):
            print(f"   Waypoint {i+1}/{len(trajectory_points_deg)}: ", end="")
            
            # Try to move to this waypoint
            success = self.move_to_position(waypoint_deg, velocity_scaling)
            
            if not success:
                print(f"❌ Trajectory blocked at waypoint {i+1}")
                return False, i
                
            print(f"✅")
            time.sleep(0.5)  # Small delay between waypoints
            
        print("✅ Complete trajectory is collision-free!\n")
        return True, None
        
    def shutdown(self):
        """Clean shutdown of the bridge"""
        if self._initialized:
            self.mover.destroy_node()
            rclpy.shutdown()
            self._initialized = False
            print("🔌 Bridge shutdown complete")


# Example usage
def example_usage():
    """
    Example showing how to use the bridge from your test scripts.
    """
    bridge = RoboDKMoveItBridge()
    
    try:
        print("\n=== Example 1: Move to home position ===")
        bridge.move_to_home()
        time.sleep(2)
        
        print("\n=== Example 2: Get current position ===")
        current = bridge.get_current_position()
        if current:
            print(f"Current joint angles: {[f'{p:.1f}°' for p in current]}")
        
        print("\n=== Example 3: Move with collision checking ===")
        # This might succeed or fail depending on your setup
        test_position = [30.0, -90.0, 90.0, -90.0, -90.0, 0.0]
        bridge.move_to_position(test_position)
        
        print("\n=== Example 4: Check a trajectory ===")
        trajectory = [
            [0.0, -90.0, 90.0, -90.0, -90.0, 0.0],    # Start
            [15.0, -90.0, 90.0, -90.0, -90.0, 0.0],   # Waypoint 1
            [30.0, -90.0, 90.0, -90.0, -90.0, 0.0],   # Waypoint 2
            [30.0, -75.0, 90.0, -90.0, -90.0, 0.0],   # Waypoint 3
        ]
        success, failed_at = bridge.check_trajectory(trajectory)
        
        if not success:
            print(f"⚠️ Trajectory has collision at waypoint {failed_at + 1}")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        bridge.shutdown()


if __name__ == '__main__':
    example_usage()
