#!/usr/bin/env python3
"""
Simple tool to move the real robot to match MoveIt's current state.
This helps synchronize the robot position before executing trajectories.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import time
import threading

class RobotSynchronizer(Node):
    def __init__(self):
        super().__init__('robot_synchronizer')
        
        self.robot_ip = "192.168.0.100"
        self.command_port = 30001
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
        
        self.moveit_joint_positions = [0.0] * 6
        self.joint_state_lock = threading.Lock()
        
        print("🔄 Robot Synchronizer - Move robot to match MoveIt position")
        print("Listening for MoveIt joint states...")

    def joint_callback(self, msg):
        """Update MoveIt joint positions"""
        with self.joint_state_lock:
            if len(msg.position) >= 6:
                self.moveit_joint_positions = list(msg.position[:6])

    def move_robot_to_moveit_position(self):
        """Move real robot to current MoveIt position"""
        with self.joint_state_lock:
            target_positions = [round(p, 4) for p in self.moveit_joint_positions]
        
        print(f"📍 Moving robot to MoveIt position: {target_positions}")
        
        try:
            # Connect to robot
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10.0)
            sock.connect((self.robot_ip, self.command_port))
            print("✅ Connected to robot")
            
            # Create URScript to move to position
            urscript = f"""
def move_to_moveit_position():
    target_q = {target_positions}
    current_q = get_actual_joint_positions()
    
    textmsg("Moving to MoveIt position...")
    
    # Calculate maximum joint difference
    max_diff = 0
    loop i=0, 6:
        diff = abs(target_q[i] - current_q[i])
        if diff > max_diff:
            max_diff = diff
        end
    end
    
    # Calculate appropriate move time (at least 3 seconds, more for large moves)
    move_time = max(3.0, max_diff * 2.0)
    
    textmsg("Move time: " + to_str(move_time) + "s")
    
    # Move to target position
    movej(target_q, a=0.3, v=0.3, t=move_time)
    
    textmsg("Reached MoveIt position!")
end

move_to_moveit_position()
"""
            
            # Send script
            sock.send(urscript.encode('utf-8'))
            print("🚀 URScript sent to robot")
            
            # Calculate wait time
            wait_time = 10.0  # Conservative wait time
            print(f"⏱️  Waiting {wait_time:.1f}s for robot to reach position...")
            
            time.sleep(wait_time)
            print("✅ Robot should now match MoveIt position!")
            
        except Exception as e:
            print(f"❌ Error: {e}")
        finally:
            try:
                sock.close()
            except:
                pass

def main():
    rclpy.init()
    
    synchronizer = RobotSynchronizer()
    
    # Wait a moment for joint states
    print("⏳ Waiting for joint state data...")
    time.sleep(2.0)
    
    try:
        # Move robot to MoveIt position
        synchronizer.move_robot_to_moveit_position()
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        synchronizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()