#!/usr/bin/env bash
set -euo pipefail

# Purpose:
# - Use the standard MoveIt launcher for UR robots (stable)
# - Still load your workstation description so test setup collisions are present
# - Keep the robot base at world origin
# - Optionally publish identity TF world->base if needed

# Configurable defaults
UR_TYPE=${UR_TYPE:-ur3e}
USE_MOCK_HARDWARE=${USE_MOCK_HARDWARE:-false}
# Set to 1 to force an identity TF from world->base
SET_WORLD_BASE_IDENTITY=${SET_WORLD_BASE_IDENTITY:-0}
# Place robot base at world origin via xacro (0/1)
BASE_AT_WORLD_ORIGIN=${BASE_AT_WORLD_ORIGIN:-0}

# Resolve workspace root (this script lives in ws_rviz/)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Source ROS 2 + this workspace (disable nounset for setup scripts)
set +u
source /opt/ros/jazzy/setup.bash
if [ -f install/setup.bash ]; then
	source install/setup.bash
fi
set -u

echo "=========================================="
echo "Launching UR MoveIt (standard) + workstation description"
echo "UR type: $UR_TYPE | mock_hw: $USE_MOCK_HARDWARE"
echo "=========================================="

# Cleanup on exit
_pids=()
cleanup() {
	echo "\nShutting down background processes..."
	for p in "${_pids[@]:-}"; do
		if kill -0 "$p" 2>/dev/null; then
			kill "$p" 2>/dev/null || true
		fi
	done
}
trap cleanup EXIT

# Find the workstation description launch file
WORKSTATION_PKG_SHARE=$(ros2 pkg prefix --share ur3e_workstation)
DESCRIPTION_LAUNCHFILE="${WORKSTATION_PKG_SHARE}/launch/workstation_description.launch.py"

echo "Using description launchfile: $DESCRIPTION_LAUNCHFILE"

# Launch ur_control with our custom description
# We pass the description_launchfile argument so ur_control launches OUR description instead of the default.
# Note: ur_control might not pass 'base_at_world_origin' to the included launch file, so it relies on the default (false).
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=$UR_TYPE \
    robot_ip:=192.168.56.106 \
    use_mock_hardware:=$USE_MOCK_HARDWARE \
    launch_rviz:=true \
    description_launchfile:="$DESCRIPTION_LAUNCHFILE" \
    &
_pids+=($!)
sleep 1

# 2) Optionally enforce world->base identity TF (use only if your URDF lacks this)
if [[ "$SET_WORLD_BASE_IDENTITY" == "1" ]]; then
	echo "Publishing static TF world->base (identity) in background..."
	ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world base_link \
		>/tmp/world_base_static_tf.log 2>&1 &
	_pids+=($!)
fi

# 3) Wait for /robot_description to be available (so MoveIt standard launcher can proceed)
echo "Waiting for /robot_description..."
timeout=30
until ros2 topic echo --once /robot_description >/dev/null 2>&1; do
	sleep 1
	((timeout--)) || { echo "Timed out waiting for /robot_description"; exit 1; }
done
echo "/robot_description received."

# 4) Kick off collision matrix publisher a bit later so MoveIt is ready
(
	sleep 8
	echo "Publishing custom collision matrix (background)..."
	ros2 run ur3e_workstation publish_collision_matrix.py \
		>/tmp/publish_collision_matrix.log 2>&1 & echo $! >/tmp/publish_collision_matrix.pid
) &

# 5) Launch the custom MoveIt launcher with RViz (loads workstation description)
echo "Starting custom MoveIt launcher (foreground)..."
set +e
ros2 launch ./custom_ur_moveit.launch.py \
    ur_type:=$UR_TYPE \
    launch_rviz:=true \
    robot_ip:=192.168.56.106 \
    use_mock_hardware:=$USE_MOCK_HARDWARE \
    base_at_world_origin:=$BASE_AT_WORLD_ORIGIN
ret=$?
set -e

exit $ret
