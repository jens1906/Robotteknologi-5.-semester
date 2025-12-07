import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.srv import GetMotionPlan, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, RobotState

import tf2_ros
import tf2_geometry_msgs
import numpy as np


class UR5MoveItClient(Node):
    def __init__(self):
        super().__init__('ur5_moveit_client')

        # Parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'tool0')
        self.declare_parameter('lift', 0.20)              # 20 cm above marker
        self.declare_parameter('execute_motion', True)    # execute after planning
        self.declare_parameter('velocity_scaling', 0.1)  # 10% of max velocity
        self.declare_parameter('acceleration_scaling', 0.1)  # 10% of max acceleration

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.ee_frame = self.get_parameter('ee_frame').get_parameter_value().string_value
        self.lift = self.get_parameter('lift').get_parameter_value().double_value
        self.execute_motion = self.get_parameter('execute_motion').get_parameter_value().bool_value
        self.velocity_scaling = self.get_parameter('velocity_scaling').get_parameter_value().double_value
        self.acceleration_scaling = self.get_parameter('acceleration_scaling').get_parameter_value().double_value

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # IO
        self.pose_sub = self.create_subscription(PoseStamped, '/aruco_pose', self.pose_cb, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.plan_srv = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.exec_act = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # State
        self.current_joints = None
        self.moved = False

        self.get_logger().info('UR5 MoveIt Client: will move tool to 20cm above first marker')

    def joint_cb(self, msg: JointState):
        names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                 'wrist_1_joint','wrist_2_joint','wrist_3_joint']
        if all(n in msg.name for n in names):
            self.current_joints = [msg.position[msg.name.index(n)] for n in names]

    def pose_cb(self, msg: PoseStamped):
        if self.moved:
            return
        try:
            # Transform marker pose into base_link using latest TF (time=0)
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
            
            # Manual transform instead of do_transform_pose
            marker_in_base = PoseStamped()
            marker_in_base.header.frame_id = self.base_frame
            marker_in_base.header.stamp = msg.header.stamp
            
            # Transform the pose manually
            marker_in_base.pose = tf2_geometry_msgs.do_transform_pose(msg.pose, tf)
            
            # Debug: Check the type and structure
            self.get_logger().info(f'marker_in_base type: {type(marker_in_base)}')
            self.get_logger().info(f'Has pose attr: {hasattr(marker_in_base, "pose")}')
            
            # Build target pose: (x,y) same, z + lift; orientation = current tool orientation
            target_pose = Pose()
            target_pose.position.x = marker_in_base.pose.position.x
            target_pose.position.y = marker_in_base.pose.position.y
            target_pose.position.z = marker_in_base.pose.position.z + self.lift

            # Convert marker orientation to desired tool orientation
            marker_quat = np.array([
                marker_in_base.pose.orientation.x,
                marker_in_base.pose.orientation.y,
                marker_in_base.pose.orientation.z,
                marker_in_base.pose.orientation.w
            ])
            
            # 90° rotation around Z-axis: [0, 0, sin(π/4), cos(π/4)] = [0, 0, 0.707, 0.707]
            rot_z_90 = np.array([0.0, 0.0, 0.7071067811865475, 0.7071067811865476])
            
            # 180° rotation around X-axis: [1, 0, 0, 0]
            rot_x_180 = np.array([1.0, 0.0, 0.0, 0.0])
            
            # Apply rotations: marker -> Z rotation -> X rotation
            intermediate_quat = self.quaternion_multiply(marker_quat, rot_z_90)
            final_quat = self.quaternion_multiply(intermediate_quat, rot_x_180)
            
            target_pose.orientation.x = final_quat[0]
            target_pose.orientation.y = final_quat[1]
            target_pose.orientation.z = final_quat[2]
            target_pose.orientation.w = final_quat[3]
            
            self.normalize_quaternion_in_place(target_pose)
            
            # Debug target position
            self.get_logger().info(f'Target position: x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}')
            
            self.plan_and_execute(target_pose)
            self.moved = True
        except Exception as e:
            self.get_logger().error(f'transform/plan error: {e}')

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions [x, y, z, w]"""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,  # x
            w1*y2 - x1*z2 + y1*w2 + z1*x2,  # y
            w1*z2 + x1*y2 - y1*x2 + z1*w2,  # z
            w1*w2 - x1*x2 - y1*y2 - z1*z2   # w
        ])

    def normalize_quaternion_in_place(self, pose: Pose):
        q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], dtype=float)
        n = np.linalg.norm(q)
        if n > 1e-9:
            pose.orientation.x = float(q[0] / n)
            pose.orientation.y = float(q[1] / n)
            pose.orientation.z = float(q[2] / n)
            pose.orientation.w = float(q[3] / n)

    def plan_and_execute(self, target_pose: Pose):
        # Use MoveIt's Cartesian path planning service
        
        # Create Cartesian path service client
        cartesian_srv = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
        if not cartesian_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('compute_cartesian_path not available')
            return

        # Get current end-effector pose
        try:
            ee_tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
            
            current_pose = Pose()
            current_pose.position.x = ee_tf.transform.translation.x
            current_pose.position.y = ee_tf.transform.translation.y
            current_pose.position.z = ee_tf.transform.translation.z
            current_pose.orientation = ee_tf.transform.rotation
            
        except Exception as e:
            self.get_logger().error(f'Could not get current pose: {e}')
            return

        # Create Cartesian path request
        cart_req = GetCartesianPath.Request()
        cart_req.header.frame_id = self.base_frame
        cart_req.group_name = 'ur_manipulator'
        cart_req.link_name = self.ee_frame
        
        # Waypoints: current pose -> target pose
        cart_req.waypoints = [current_pose, target_pose]
        
        # Path parameters
        cart_req.max_step = 0.01  # 1cm steps for smooth motion
        cart_req.jump_threshold = 0.0  # No joint jumps allowed
        cart_req.avoid_collisions = True
        
        # Start state from current joints if available
        if self.current_joints is not None:
            rs = RobotState()
            js = JointState()
            js.name = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                       'wrist_1_joint','wrist_2_joint','wrist_3_joint']
            js.position = self.current_joints
            rs.joint_state = js
            cart_req.start_state = rs

        # Debug info
        self.get_logger().info(f'Current pose: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}')
        self.get_logger().info(f'Target pose: x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}')

        fut = cartesian_srv.call_async(cart_req)
        fut.add_done_callback(self.on_cartesian_plan)

    def on_cartesian_plan(self, fut):
        try:
            resp = fut.result()
            
            # Check if path was successfully computed
            if resp.fraction < 0.95:  # Less than 95% of path computed
                self.get_logger().error(f'Cartesian path planning failed: only {resp.fraction*100:.1f}% computed')
                return
                
            self.get_logger().info(f'Cartesian path computed: {resp.fraction*100:.1f}% successful')
            
            traj = resp.solution
            
            # Apply velocity scaling to the trajectory
            self.scale_trajectory_velocity(traj, self.velocity_scaling)

            if not self.execute_motion:
                self.get_logger().info('Cartesian plan ok (execution disabled)')
                return

            if not self.exec_act.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('execute_trajectory not available')
                return

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = traj
            send = self.exec_act.send_goal_async(goal)
            send.add_done_callback(self.on_exec_sent)
            
        except Exception as e:
            self.get_logger().error(f'Cartesian plan callback error: {e}')

    def scale_trajectory_velocity(self, trajectory, scale_factor):
        """Scale the velocity and acceleration of trajectory points"""
        for point in trajectory.joint_trajectory.points:
            if hasattr(point, 'velocities') and point.velocities:
                point.velocities = [v * scale_factor for v in point.velocities]
            if hasattr(point, 'accelerations') and point.accelerations:
                point.accelerations = [a * scale_factor for a in point.accelerations]
            
            # Scale time correctly - convert to total nanoseconds first, then scale, then convert back
            total_nanosec = point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec
            scaled_total_nanosec = int(total_nanosec / scale_factor)
            
            point.time_from_start.sec = scaled_total_nanosec // 1_000_000_000
            point.time_from_start.nanosec = scaled_total_nanosec % 1_000_000_000

    def on_exec_sent(self, fut):
        try:
            handle = fut.result()
            if not handle.accepted:
                self.get_logger().error('execution rejected')
                return
            res_fut = handle.get_result_async()
            res_fut.add_done_callback(self.on_exec_done)
        except Exception as e:
            self.get_logger().error(f'exec send error: {e}')

    def on_exec_done(self, fut):
        try:
            res = fut.result().result
            self.get_logger().info(f'execution code: {res.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'exec result error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UR5MoveItClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
