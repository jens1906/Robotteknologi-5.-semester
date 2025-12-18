# System Test Evidence from December 5, 2025 Rosbags

This document summarizes the four rosbag recordings located in `TestOfInterface/`. They represent the only available data for validating the user-interface-driven MoveIt workflow at Vattenfall.

## Bag inventory

| Bag dir | Start (local) | Duration | Messages | Notes |
| --- | --- | --- | --- | --- |
| `1. rosbag` | 2025-12-05 12:25:39 | 173.1 s | 344,632 | Dense robot telemetry; minimal UI activity |
| `2. rosbag` | 2025-12-05 12:35:34 | 148.2 s | 271,495 | Similar to bag 1; controller-only traffic |
| `3. rosbag` | 2025-12-05 12:57:03 | 257.1 s | 441,393 | Longest stable window; rich IO streams |
| `4. rosbag` | 2025-12-05 13:12:37 | 292.9 s | 571,213 | Highest-rate run; still telemetry-only |

## Core telemetry rates

Average message rates for the principal robot topics show the UR driver and broadcasters were healthy for each recording window.

| Bag | `/joint_states` | `/tcp_pose_broadcaster/pose` | `/force_torque_sensor_broadcaster/wrench` | `/speed_scaling_state_broadcaster/speed_scaling` |
| --- | --- | --- | --- | --- |
| 1 | ~200 Hz (34,672 msgs) | ~197 Hz (34,066 msgs) | ~200 Hz (34,694 msgs) | ~203 Hz (35,111 msgs) |
| 2 | ~201 Hz (29,735 msgs) | ~204 Hz (30,272 msgs) | ~202 Hz (29,981 msgs) | ~205 Hz (30,398 msgs) |
| 3 | ~216 Hz (55,403 msgs) | ~215 Hz (55,191 msgs) | ~214 Hz (55,109 msgs) | ~215 Hz (55,129 msgs) |
| 4 | ~227 Hz (66,457 msgs) | ~228 Hz (66,915 msgs) | ~225 Hz (66,019 msgs) | ~229 Hz (67,066 msgs) |

## UI/perception coverage

- `/corrosion/thresholding_pub` appears in every bag (341–767 messages), proving the RealSense feed and threshold view were active.
- `/ui/corrosion_area_{add,remove}_pub` only fire 1–4 times per run, indicating very few paint/erase actions were recorded.
- `/ui/corrosion_area_accept_pub` fires exactly once per bag, matching the operator acknowledging a corrosion mask.
- `/ui/emergency_stop_pub`, `/ui/home_position_pub`, and `/ui/terminate_pub` never fire in these recordings; safety interactions are unverified.
- Topics such as `/corrosion/workspace`, `/corrosion/pointcloud_rviz`, and `/corrosion/tool_size` either never publish or only emit a single sample, so higher-level perception and planning steps were not exercised.

## Missing Servo evidence

Despite the GUI changes, none of the four rosbags contain:

- `/servo_node/delta_twist_cmds`
- `/servo_node/{pause_servo,unpause_servo}`
- Any `geometry_msgs/TwistStamped` topic carrying joystick or Z-button commands

This strongly suggests MoveIt Servo was either not running on the driver PC, or the bag capture happened on a different machine that never saw the Servo node via DDS discovery. Because of this gap, the current recordings cannot prove joystick/Z jogging functionality.

## Conclusions for the system test

1. **Robot telemetry is well documented** — Each bag provides 2–5 minutes of continuous TF, joint states, TCP pose, IO states, force/torque, and speed scaling data at 200–230 Hz.
2. **Operator workflow coverage is thin** — Only a handful of paint/erase events and one "Accept" action were captured per run; no emergency-stop, homing, or terminate actions are present.
3. **Jogging loop lacks evidence** — Without Servo twist commands, there is no proof that the GUI-driven MoveIt Servo integration worked during these tests.
4. **Perception pipeline mostly idle** — Advanced corrosion topics barely publish, so the test does not exercise full vision-to-path planning.

## Recommended follow-ups

1. **Record from the GUI workstation** so `/servo_node/*` and all `/ui/*` topics are captured alongside robot telemetry.
2. **Trigger every safety action once per run** (Home, Z±, E-stop, Terminate) while recording.
3. **Add ROS graph visibility checks** (`ros2 service list | grep servo_node`, `ros2 topic echo /servo_node/status`) before each bag to ensure DDS visibility.
4. **Optionally capture `/rosout` and diagnostics** to correlate warnings (e.g., Servo service missing) with robot state.

These steps will give future system tests end-to-end evidence that the GUI, Servo jog loop, and safety functions collectively operate on the UR3e platform.

## Visual artifacts generated (Dec 5 analysis refresh)

The helper script `scripts/extract_threshold_and_orientation.py` now accepts `--last-threshold-only` and automatically plots `/tool_orientation/path` waypoints when that topic is present. Running

```
python3 TestOfInterface/scripts/extract_threshold_and_orientation.py \
	--bag "TestOfInterface/<N>. rosbag" \
	--out-dir TestOfInterface/plots \
	--last-threshold-only
```

produced one corrosion-threshold PNG per bag (final frame) plus tool-path orientation plots for the bags that contained `/tool_orientation/path`. An additional console report compares the first commanded waypoint against the closest recorded TCP pose sample to highlight orientation deltas.

### How to read the plots

- **Threshold PNG (`*_threshold_00.png`)** – This is the *final* binary corrosion mask seen by the GUI operator for that run. White pixels denote regions classified as corrosion after RealSense preprocessing and thresholding; black pixels are rejected areas. Comparing these images between bags shows how mask refinement evolved over the session.
- **TCP orientation plot (`*_tcp_orientation.png`)** – A two-panel figure. The top subplot charts quaternion components (`qw`, `qx`, `qy`, `qz`) over time, revealing any sudden changes in tool attitude. The bottom subplot converts the same data to roll/pitch/yaw (XYZ intrinsic) to make drift or spikes visually obvious. Long flat sections mean the TCP held a steady pose; rapid swings indicate robot motion or external disturbances.
- **Tool-path orientation plot (`*_tool_path_orientation.png`)** – Also a two-panel figure but indexed by waypoint order rather than time. It records every orientation contained in `/tool_orientation/path`, i.e., the commanded corrosion scanning trajectory. The top subplot shows quaternion values per waypoint, while the bottom subplot shows the corresponding Euler angles. This lets downstream analysis verify that the command queue was sensible (e.g., smooth yaw sweep) even if the robot did something different.
- **Orientation delta log** – The table below lists the difference between the *first* commanded waypoint and the closest sampled TCP pose. Large yaw deltas (>3 rad) imply the commanded orientation never reached the robot during the captured interval, reinforcing the missing Servo evidence noted earlier.

| Bag | Threshold PNG | Tool-path PNG | Orientation deltas (roll / pitch / yaw) |
| --- | --- | --- | --- |
| 1 | `plots/1._rosbag_threshold_00.png` | `plots/1._rosbag_tool_path_orientation.png` | `-0.004 / +0.082 / +3.221 rad`
| 2 | `plots/2._rosbag_threshold_00.png` | `plots/2._rosbag_tool_path_orientation.png` | `-0.023 / +0.070 / +3.238 rad`
| 3 | `plots/3._rosbag_threshold_00.png` | `plots/3._rosbag_tool_path_orientation.png` | `-0.035 / +0.085 / -3.042 rad`
| 4 | `plots/4._rosbag_threshold_00.png` | _Topic absent – no plot_ | _Topic absent – no comparison_

> Large yaw differences reflect the mismatch between the static path definition (recorded once per bag) and the real TCP trajectory, reinforcing the need to capture live Servo commands in future tests.
