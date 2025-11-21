#!/usr/bin/env python3
"""
Script to properly initialize and start the UR robot for movement
"""
import socket
import time

def initialize_robot_for_movement():
    robot_ip = '192.168.0.100'
    
    print("=== UR Robot Initialization for Movement ===")
    
    # Connect to dashboard
    print("\n1. Connecting to robot dashboard...")
    try:
        dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        dashboard.settimeout(10.0)
        dashboard.connect((robot_ip, 29999))
        print("   ✓ Connected to dashboard")
        
        # Check initial status
        print("\n2. Checking robot status...")
        dashboard.send(b'safetymode\n')
        safety_status = dashboard.recv(1024).decode('utf-8').strip()
        print(f"   Safety Mode: {safety_status}")
        
        dashboard.send(b'robotmode\n')
        robot_status = dashboard.recv(1024).decode('utf-8').strip()
        print(f"   Robot Mode: {robot_status}")
        
        dashboard.send(b'programState\n')
        program_status = dashboard.recv(1024).decode('utf-8').strip()
        print(f"   Program State: {program_status}")
        
        # Try to start the robot if it's stopped
        if "STOPPED" in safety_status:
            print("\n3. Robot is STOPPED - attempting to start...")
            
            # Unlock protective stop (if any)
            print("   Sending unlock_protective_stop command...")
            dashboard.send(b'unlock_protective_stop\n')
            response = dashboard.recv(1024).decode('utf-8').strip()
            print(f"   Response: {response}")
            time.sleep(1)
            
            # Close safety popup (if any)
            print("   Sending close_safety_popup command...")
            dashboard.send(b'close_safety_popup\n')
            response = dashboard.recv(1024).decode('utf-8').strip()
            print(f"   Response: {response}")
            time.sleep(1)
            
            # Start the robot
            print("   Sending start command...")
            dashboard.send(b'start\n')
            response = dashboard.recv(1024).decode('utf-8').strip()
            print(f"   Response: {response}")
            time.sleep(2)
            
        # Check status after initialization
        print("\n4. Checking status after initialization...")
        dashboard.send(b'safetymode\n')
        safety_status = dashboard.recv(1024).decode('utf-8').strip()
        print(f"   Safety Mode: {safety_status}")
        
        dashboard.send(b'robotmode\n')
        robot_status = dashboard.recv(1024).decode('utf-8').strip()
        print(f"   Robot Mode: {robot_status}")
        
        dashboard.close()
        
    except Exception as e:
        print(f"   ✗ Dashboard error: {e}")
        return False
    
    # Test movement with properly formatted URScript
    print("\n5. Testing movement with initialized robot...")
    try:
        cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cmd_socket.settimeout(10.0)
        cmd_socket.connect((robot_ip, 30001))
        
        # Send a simple movement test
        urscript = """def movement_test():
  textmsg("Robot initialized - testing movement")
  
  # Get current joint positions
  start_q = get_actual_joint_positions()
  textmsg("Current position obtained")
  
  # Create target position with small movement in joint 0
  target_q = [start_q[0] + 0.05, start_q[1], start_q[2], start_q[3], start_q[4], start_q[5]]
  
  textmsg("Moving to target position")
  movej(target_q, a=0.2, v=0.1, t=0, r=0)
  sleep(1)
  
  textmsg("Returning to start position")
  movej(start_q, a=0.2, v=0.1, t=0, r=0)
  sleep(1)
  
  textmsg("Movement test completed successfully")
end

movement_test()
"""
        
        print("   Sending movement test URScript...")
        cmd_socket.send(urscript.encode('utf-8'))
        
        # Check for response
        time.sleep(1)
        try:
            cmd_socket.settimeout(2.0)
            response = cmd_socket.recv(1024)
            if response:
                print(f"   Robot response: {response.decode('utf-8', errors='ignore')}")
        except socket.timeout:
            print("   No immediate response (normal)")
        
        print("   Waiting 8 seconds for movement test...")
        time.sleep(8)
        
        cmd_socket.close()
        print("   ✓ Movement test sent")
        
    except Exception as e:
        print(f"   ✗ Movement test error: {e}")
        return False
    
    print("\n=== Initialization Complete ===")
    print("The robot should now be ready for MoveIt control.")
    print("If you saw movement, the robot is working properly.")
    print("If no movement, check:")
    print("- Emergency stop is not pressed")
    print("- Robot is in Remote Control mode on the teach pendant") 
    print("- No safety violations or protective stops")
    
    return True

if __name__ == '__main__':
    initialize_robot_for_movement()