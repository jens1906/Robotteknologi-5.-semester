#!/usr/bin/env python3
"""
Advanced test script to diagnose and fix robot movement issues
"""
import socket
import time
import threading

def test_robot_status_and_movement():
    robot_ip = '192.168.0.100'
    
    print("=== Advanced Robot Test & Diagnosis ===")
    
    # Test 1: Dashboard connection (port 29999)
    print("\n1. Testing Dashboard Connection...")
    try:
        dashboard_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        dashboard_socket.settimeout(5.0)
        dashboard_socket.connect((robot_ip, 29999))
        
        # Check robot mode
        dashboard_socket.send(b'robotmode\n')
        response = dashboard_socket.recv(1024).decode('utf-8').strip()
        print(f"   Robot Mode: {response}")
        
        # Check program state
        dashboard_socket.send(b'programState\n')
        response = dashboard_socket.recv(1024).decode('utf-8').strip()
        print(f"   Program State: {response}")
        
        # Check safety mode
        dashboard_socket.send(b'safetymode\n')
        response = dashboard_socket.recv(1024).decode('utf-8').strip()
        print(f"   Safety Mode: {response}")
        
        # Check if robot is in Remote Control mode
        dashboard_socket.send(b'is_in_remote_control\n')
        response = dashboard_socket.recv(1024).decode('utf-8').strip()
        print(f"   Remote Control: {response}")
        
        dashboard_socket.close()
        print("   ✓ Dashboard connection successful")
    except Exception as e:
        print(f"   ✗ Dashboard error: {e}")
        return False
    
    # Test 2: Real-time interface (port 30003)
    print("\n2. Testing Real-time Interface...")
    try:
        rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rt_socket.settimeout(5.0)
        rt_socket.connect((robot_ip, 30003))
        
        # Read one packet
        data = rt_socket.recv(1108)  # Standard RT packet size
        if len(data) >= 1108:
            print("   ✓ Real-time data received")
        else:
            print(f"   ! Partial data received: {len(data)} bytes")
        
        rt_socket.close()
    except Exception as e:
        print(f"   ✗ Real-time interface error: {e}")
    
    # Test 3: Command interface with improved URScript
    print("\n3. Testing Command Interface with Movement...")
    try:
        cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cmd_socket.settimeout(10.0)
        cmd_socket.connect((robot_ip, 30001))
        
        # Send improved URScript
        urscript = '''def test_movement():
  textmsg("Starting robot test movement")
  
  # Get current position
  current_joints = get_actual_joint_positions()
  textmsg("Current position obtained")
  
  # Small movement test - move joint 0 by 0.1 radians
  target_joints = [current_joints[0] + 0.1, current_joints[1], current_joints[2], current_joints[3], current_joints[4], current_joints[5]]
  
  textmsg("Moving to test position")
  movej(target_joints, a=0.3, v=0.1, t=0, r=0)
  
  textmsg("Waiting 2 seconds")
  sleep(2)
  
  textmsg("Returning to original position")
  movej(current_joints, a=0.3, v=0.1, t=0, r=0)
  
  textmsg("Test movement completed")
end

test_movement()
'''
        
        print("   Sending URScript:")
        print("   " + "\n   ".join(urscript.split('\n')[:5]) + "...")
        
        # Send script
        urscript_bytes = urscript.encode('utf-8')
        cmd_socket.send(urscript_bytes)
        
        # Try to read any response
        time.sleep(1)
        try:
            cmd_socket.settimeout(2.0)
            response = cmd_socket.recv(1024)
            if response:
                print(f"   Robot response: {response.decode('utf-8', errors='ignore')}")
            else:
                print("   No immediate response")
        except socket.timeout:
            print("   No response (timeout - this is normal)")
        
        print("   ✓ URScript sent successfully")
        print("   Waiting 10 seconds for movement...")
        time.sleep(10)
        
        cmd_socket.close()
        
    except Exception as e:
        print(f"   ✗ Command interface error: {e}")
        return False
    
    print("\n=== Test Complete ===")
    print("Check if you saw any movement on the robot.")
    print("If no movement occurred, the robot may need to be:")
    print("- Put in Remote Control mode")
    print("- Have a program loaded/running")
    print("- Be in the correct safety state")
    
    return True

if __name__ == '__main__':
    test_robot_status_and_movement()