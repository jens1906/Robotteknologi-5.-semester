#!/usr/bin/env python3
"""
Simple test script to verify URScript communication with UR robot
"""
import socket
import time

def test_robot_connection():
    robot_ip = '192.168.0.100'
    command_port = 30001
    
    print(f"Testing connection to UR robot at {robot_ip}:{command_port}")
    
    try:
        # Connect to robot
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0)
        sock.connect((robot_ip, command_port))
        print("✓ Connected to robot command interface")
        
        # Test simple URScript command
        test_script = '''textmsg("Test message from Python")
movej(get_actual_joint_positions(), a=0.5, v=0.3, t=2, r=0)
textmsg("Test movement completed")
'''
        
        print("Sending test URScript:")
        print(test_script)
        
        # Send script
        sock.send(test_script.encode('utf-8'))
        print("✓ URScript sent to robot")
        
        # Try to read response
        try:
            sock.settimeout(2.0)
            response = sock.recv(1024)
            if response:
                print(f"Robot response: {response.decode('utf-8', errors='ignore')}")
        except socket.timeout:
            print("No immediate response (this is normal)")
        
        print("Waiting 5 seconds for execution...")
        time.sleep(5)
        
        sock.close()
        print("✓ Test completed")
        
    except Exception as e:
        print(f"✗ Error: {e}")

if __name__ == '__main__':
    test_robot_connection()