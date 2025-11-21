#!/usr/bin/env python3
import socket
import time

def test_ur_script_execution():
    """Simple test to verify URScript execution"""
    
    robot_ip = "192.168.0.100"
    command_port = 30001
    
    print("Testing URScript execution...")
    
    try:
        # Connect to robot
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0)
        sock.connect((robot_ip, command_port))
        print(f"✓ Connected to robot at {robot_ip}:{command_port}")
        
        # Simple test script - just move one joint slightly
        test_script = """
def test_move():
    # Get current joint positions
    current_q = get_actual_joint_positions()
    textmsg("Starting test movement...")
    
    # Create target position (move shoulder pan by 0.1 radians)
    target_q = [current_q[0] + 0.1, current_q[1], current_q[2], current_q[3], current_q[4], current_q[5]]
    
    # Move to target
    movej(target_q, a=0.2, v=0.2, t=2.0)
    
    # Move back to original
    movej(current_q, a=0.2, v=0.2, t=2.0)
    
    textmsg("Test movement completed!")
end

test_move()
"""
        
        print("Sending test URScript...")
        print("URScript content:")
        print(test_script)
        
        # Send the script
        sock.send((test_script + "\n").encode('utf-8'))
        
        print("✓ URScript sent successfully")
        print("⚠️  Watch the robot - it should move slightly and return to original position")
        print("You should also see messages on the robot teach pendant")
        
        # Wait for execution
        time.sleep(8)
        
        print("Test completed!")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        
    finally:
        try:
            sock.close()
        except:
            pass

if __name__ == "__main__":
    print("========================================")
    print("UR3e URScript Execution Test")
    print("========================================")
    print()
    print("This will test if URScript commands work.")
    print("The robot should make a small movement.")
    print()
    input("Press Enter to continue (or Ctrl+C to abort)...")
    
    test_ur_script_execution()