#!/usr/bin/env python3
"""
Simple script to move UR robot to a safe position for MoveIt planning
"""

import socket
import time

def move_robot_to_safe_position():
    robot_ip = "192.168.0.100"
    port = 30001
    
    print("Moving UR3e to safe position for MoveIt...")
    print("This will fix the 'Start state out of bounds' error")
    
    # Simple safe position - all joints away from limits
    urscript = """
def move_to_safe():
    # Safe position: slightly bent arm, away from joint limits
    safe_pos = [0.5, -1.0, 1.2, -1.7, -1.57, 0.0]
    
    textmsg("Moving to MoveIt-safe position...")
    movej(safe_pos, a=0.5, v=0.3, t=0, r=0)
    textmsg("Safe position reached - MoveIt ready!")
end

move_to_safe()
"""
    
    try:
        print("Connecting to robot...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)
        sock.connect((robot_ip, port))
        
        print("Sending movement command...")
        sock.send(urscript.encode('utf-8'))
        
        print("Command sent! Robot should be moving to safe position...")
        print("Wait about 5-10 seconds for movement to complete.")
        print("Then try planning in MoveIt again.")
        
        sock.close()
        return True
        
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure robot is:")
        print("- Powered on")
        print("- In Remote mode") 
        print("- No protective stop active")
        return False

if __name__ == '__main__':
    move_robot_to_safe_position()