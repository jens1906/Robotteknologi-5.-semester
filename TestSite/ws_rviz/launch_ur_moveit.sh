#!/usr/bin/env bash
set -euo pipefail

# Purpose:
# - Use the standard MoveIt launcher for UR robots (stable)
# - Still load your workstation description so test setup collisions are present
# - Keep the robot base at world origin
# - Optionally publish identity TF world->base if needed

# Configurable defaults
UR_TYPE=${UR_TYPE:-ur3e}
USE_MOCK_HARDWARE=${USE_MOCK_HARDWARE:-true}
# Enable MoveIt Servo by default (can override via env or CLI)
LAUNCH_SERVO=${LAUNCH_SERVO:-false}
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

# 1) Start workstation description publisher so /robot_description exists
#    This includes your test-setup geometry via ur3e_workstation/urdf/workstation.xacro
echo "Starting workstation description publisher (background)..."
ros2 launch ur3e_workstation workstation_description.launch.py \
	ur_type:=$UR_TYPE \
	use_mock_hardware:=$USE_MOCK_HARDWARE \
	base_at_world_origin:=$([ "$BASE_AT_WORLD_ORIGIN" = "1" ] && echo true || echo false) \
	>/tmp/workstation_description.log 2>&1 &
_pids+=($!)

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
	echo "Publishing environment collision objects (background)..."
	ros2 run ur3e_workstation publish_environment.py \
		>/tmp/publish_environment.log 2>&1
	
	echo "Publishing custom collision matrix (background)..."
	ros2 run ur3e_workstation publish_collision_matrix.py \
		>/tmp/publish_collision_matrix.log 2>&1 & echo $! >/tmp/publish_collision_matrix.pid
) &

# 5) Launch the custom MoveIt launcher with RViz (using correct SRDF)
echo "Starting custom MoveIt launcher (foreground)..."
set +e
ros2 launch ur3e_workstation custom_moveit.launch.py \
    ur_type:=$UR_TYPE \
    use_mock_hardware:=$USE_MOCK_HARDWARE "$@"
ret=$?
set -e

exit $ret
