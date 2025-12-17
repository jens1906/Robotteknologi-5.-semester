#!/usr/bin/env python3
"""Manual regression test for the tool_orientation module.

This script spins up the ToolOrientationNode directly, publishes a 30-point
sine-wave toolpath (matching the validation scenario described in the report),
waits for the PoseArray result, and renders the outcome as a JPG file showing
the adjusted positions plus representative orientation axes.

Usage (after sourcing your ROS 2 workspace):
    python -m tool_orientation.sine_path_orientation_test

The image is saved under ``tool_orientation/test_results/`` relative to this
package directory (created automatically).

I need to replicate test of the tool orientation module, and i need it to save the visual result as an jpg image. Perhaps like the prior graphs were we can see the output quaternion orientations if that makes sense. The module test to replicate: The module was tested to verify correct computation and publication of tool orientations for a 30-point curved path (sine wave with Z variation). The test objective is to verify subscription to input path points, correct rotation matrix computation, and publication on the output topic. Setup is:
\begin{itemize}
    \item Input topic: \texttt{/parameterization/xyz\_path} (Float64MultiArray)
    \item Output topic: \texttt{/tool\_orientation/path} (geometry\_msgs/PoseArray.msg)
    \item Test data: 30-point sine-wave path
\end{itemize}

%Show figure here!

Results showed that the node initialised correctly, all rotation matrices computed and published, and determinants $\approx 1$ and thus solving \ref{Scoped Requirement 2.3}. Consecutive orientations were along the path. The outputs are positions (N,3) and quaternions (qw,qx,qy,qz) for the given path.

The module computes and publishes valid end-effector orientations, handles path configurations, and ensures motion along curved paths. 
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use('Agg')  # Ensure headless environments can write image files
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import ByteMultiArray, Float64MultiArray

from .tool_orientation_node import ToolOrientationNode


def _build_sine_wave_path(num_points: int = 30) -> np.ndarray:
    """Return Nx3 array in millimetres following a curved (sine) path."""
    t = np.linspace(0.0, 2.0 * math.pi, num_points)
    radius = 150.0  # mm
    x = radius * np.cos(t)
    y = radius * np.sin(t)
    z = 40.0 * np.sin(0.5 * t) + 80.0  # introduce Z undulation
    return np.column_stack((x, y, z))


def _build_surface_flags(length: int) -> np.ndarray:
    """Create boolean-on-surface flags, last quarter marked as off-surface."""
    flags = np.ones(length, dtype=np.uint8)
    off_surface_start = max(0, length - length // 4)
    flags[off_surface_start:] = 0
    return flags


class ToolOrientationTestHarness(Node):
    """Publishes the sine path and captures PoseArray output."""

    def __init__(self) -> None:
        super().__init__('tool_orientation_test_harness')
        self.pose_msg: PoseArray | None = None
        self.test_complete = False
        self.publish_count = 0
        self.max_publishes = 3

        self.path_points = _build_sine_wave_path()
        self.on_surface_flags = _build_surface_flags(len(self.path_points))

        self.pose_sub = self.create_subscription(
            PoseArray,
            '/tool_orientation/path',
            self.pose_callback,
            10,
        )
        self.path_pub = self.create_publisher(
            Float64MultiArray,
            '/parameterization/xyz_path',
            10,
        )
        self.surface_pub = self.create_publisher(
            ByteMultiArray,
            '/path/on_surface',
            10,
        )
        self.timer = self.create_timer(0.5, self.publish_test_data)

    def publish_test_data(self) -> None:
        if self.test_complete or self.publish_count >= self.max_publishes:
            return

        path_msg = Float64MultiArray()
        path_msg.data = self.path_points.flatten().tolist()

        surface_msg = ByteMultiArray()
        surface_msg.data = self.on_surface_flags.tolist()

        self.path_pub.publish(path_msg)
        self.surface_pub.publish(surface_msg)
        self.publish_count += 1
        self.get_logger().info(f'Published sine path sample #{self.publish_count}')

    def pose_callback(self, msg: PoseArray) -> None:
        if self.test_complete:
            return
        self.pose_msg = msg
        self.test_complete = True
        self.get_logger().info(
            f'Received PoseArray with {len(msg.poses)} poses from tool_orientation_node'
        )


def _posearray_to_arrays(msg: PoseArray) -> dict[str, np.ndarray]:
    positions = np.array(
        [[p.position.x, p.position.y, p.position.z] for p in msg.poses],
        dtype=float,
    )
    quaternions = np.array(
        [[p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
         for p in msg.poses],
        dtype=float,
    )
    return {'positions': positions, 'quaternions': quaternions}


def _generate_visualization(positions: np.ndarray,
                             quaternions: np.ndarray,
                             output_path: Path) -> Dict[str, float]:
    rotations = R.from_quat(quaternions)
    determinants = np.array([np.linalg.det(rot.as_matrix()) for rot in rotations])

    # Down-sample orientation vectors for plotting clarity
    sample_count = min(len(positions), 10)
    sample_idx = np.linspace(0, len(positions) - 1, sample_count, dtype=int)
    sample_positions = positions[sample_idx]
    sample_rotations = rotations[sample_idx]
    tool_x_axes = sample_rotations.apply(np.tile([0.05, 0.0, 0.0], (sample_count, 1)))
    tool_z_axes = sample_rotations.apply(np.tile([0.0, 0.0, 0.05], (sample_count, 1)))

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], '-o', label='Adjusted path')
    ax.quiver(
        sample_positions[:, 0], sample_positions[:, 1], sample_positions[:, 2],
        tool_x_axes[:, 0], tool_x_axes[:, 1], tool_x_axes[:, 2],
        color='tab:red', label='Tool X axis'
    )
    ax.quiver(
        sample_positions[:, 0], sample_positions[:, 1], sample_positions[:, 2],
        tool_z_axes[:, 0], tool_z_axes[:, 1], tool_z_axes[:, 2],
        color='tab:blue', label='Tool Z axis'
    )
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('Tool Orientation Sine-Path Validation')
    ax.legend(loc='upper right')
    ax.view_init(elev=25.0, azim=40.0)
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)

    return {
        'min_det': float(np.min(determinants)),
        'max_det': float(np.max(determinants)),
        'mean_det': float(np.mean(determinants)),
    }


def run_orientation_regression() -> Dict[str, float]:
    rclpy.init()
    tool_node = ToolOrientationNode()
    tester = ToolOrientationTestHarness()
    executor = MultiThreadedExecutor()
    executor.add_node(tool_node)
    executor.add_node(tester)

    timeout_s = 10.0
    start_time = tester.get_clock().now()
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            if tester.test_complete:
                break
            elapsed = (tester.get_clock().now() - start_time).nanoseconds * 1e-9
            if elapsed > timeout_s:
                raise TimeoutError('Timed out waiting for PoseArray message.')
    finally:
        tester.destroy_node()
        tool_node.destroy_node()
        rclpy.shutdown()

    arrays = _posearray_to_arrays(tester.pose_msg)  # type: ignore[arg-type]
    positions = arrays['positions']
    quaternions = arrays['quaternions']
    # Convert mm -> meters (ToolOrientationNode already outputs meters, but keep explicit)

    package_root = Path(__file__).resolve().parent
    output_path = package_root / 'test_results' / 'tool_orientation_sine_test.jpg'
    stats = _generate_visualization(positions, quaternions, output_path)
    stats['output_path'] = str(output_path)
    stats['num_points'] = positions.shape[0]
    return stats


if __name__ == '__main__':
    results = run_orientation_regression()
    print('Tool orientation sine-path regression complete:')
    print(f"  Pose count: {results['num_points']}")
    print(f"  Determinant range: {results['min_det']:.6f} – {results['max_det']:.6f}")
    print(f"  Image saved to: {results['output_path']}")
