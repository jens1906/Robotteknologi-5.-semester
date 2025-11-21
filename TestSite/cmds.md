# open rviz to robot:
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  initial_joint_controller:=scaled_joint_trajectory_controller

# open motion planner
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  launch_rviz:=true
