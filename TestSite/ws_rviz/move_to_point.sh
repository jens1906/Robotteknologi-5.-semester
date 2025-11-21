#!/bin/bash
# Simple script to move robot to a single point using MoveIt

cd /home/andr465m/Documents/GitHub/Robotteknologi-5.-semester/TestSite/ws_rviz
source install/setup.bash

echo "=========================================="
echo "Move Robot to Single Point"
echo "=========================================="
echo ""
echo "This will move the robot to a single target pose."
echo "Make sure launch_ur_moveit.sh is running!"
echo ""

# Simple Python script to move to one point
python3 << 'PYTHON_EOF'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import sys

rclpy.init()
node = Node('simple_move_test')

print("\n" + "="*60)
print("Testing simple movement to one point")
print("="*60)

# Create a simple target pose - straight above the robot base
target_pose = PoseStamped()
target_pose.header.frame_id = "world"
target_pose.header.stamp = node.get_clock().now().to_msg()

# Position: slightly in front and above the robot
target_pose.pose.position.x = float(0.0)
target_pose.pose.position.y = float(-0.6)
target_pose.pose.position.z = float(0.25)

# Orientation: pointing down (simple quaternion)
target_pose.pose.orientation.w = float(1.0)  # No rotation
target_pose.pose.orientation.x = float(0.0)
target_pose.pose.orientation.y = float(0.0)
target_pose.pose.orientation.z = float(0.0)

print(f"\nTarget position: [{target_pose.pose.position.x}, {target_pose.pose.position.y}, {target_pose.pose.position.z}]")
print(f"Target orientation: w={target_pose.pose.orientation.w}")

# Call IK service to check if position is reachable
ik_client = node.create_client(GetPositionIK, '/compute_ik')
print("\nWaiting for IK service...")

if not ik_client.wait_for_service(timeout_sec=5.0):
    print("ERROR: IK service not available!")
    print("Make sure launch_ur_moveit.sh is running.")
    sys.exit(1)

print("✓ IK service available")

# Create IK request
from moveit_msgs.srv import GetPositionIK
request = GetPositionIK.Request()
request.ik_request.group_name = "ur_manipulator"
request.ik_request.pose_stamped = target_pose
request.ik_request.timeout.sec = 5
request.ik_request.avoid_collisions = True

print("\nCalling IK service to check if target is reachable...")

future = ik_client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

if future.result() is not None:
    response = future.result()
    error_code = response.error_code.val
    
    if error_code == MoveItErrorCodes.SUCCESS:
        print("✓ SUCCESS! Target position is reachable!")
        print(f"  Joint solution found:")
        for i, (name, pos) in enumerate(zip(response.solution.joint_state.name[:6], 
                                             response.solution.joint_state.position[:6])):
            print(f"    {name}: {pos:.3f} rad")
        print("\nNow you can use MoveIt RViz panel to move to this position:")
        print("  1. In RViz Motion Planning panel")
        print("  2. Set Goal State manually or use interactive markers")
        print("  3. Click 'Plan' then 'Execute'")
    else:
        print(f"✗ FAILED! Error code: {error_code}")
        print("  Target position is NOT reachable by the robot")
        print("  Try adjusting the position closer to robot base")
else:
    print("✗ Service call failed or timed out")

node.destroy_node()
rclpy.shutdown()

PYTHON_EOF

echo ""
echo "Test complete!"
