import sys
import rclpy
import cv2 as cv
import threading
import numpy as np
import message_filters
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from pathlib import Path
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import Log
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from PyQt6.QtGui import QImage, QPixmap, QFont
from user_interface.GUI import Ui_MainWindow
from PyQt6.QtCore import pyqtSignal, QObject, Qt
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QMessageBox, QTextEdit
from user_interface.joystick import Joystick
import signal

Test = True
showImages = True
printlogger = False



class RosSignalEmitter(QObject):
    data_signal = pyqtSignal(str)           
    image_signal = pyqtSignal(object)
    painting_color_signal = pyqtSignal(str)  # Signal to change painting button colors
    log_signal = pyqtSignal(str)  # Signal for ROS log messages to display in GUI       

class UserInterfaceNode(Node):
    def __init__(self, signal_emitter, ui_instance=None):
        super().__init__('user_interface')
        self.signal_emitter = signal_emitter
        self.ui_instance = ui_instance
        
        # Wrap logger methods to also emit to GUI log_signal
        self._wrap_logger_for_gui()
        
        # QoS profile for image topics (reliable to match RealSense camera settings)
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.ui_terminate_pub = self.create_publisher(Bool, '/ui/terminate_pub', 10)
        self.ui_home_position_pub = self.create_publisher(Bool, '/ui/home_position_pub', 10)
        self.ui_emergency_stop_pub = self.create_publisher(Bool, '/ui/emergency_stop_pub', 10)
        self.ui_corrosion_area_add_pub = self.create_publisher(Image, '/ui/corrosion_area_add_pub', image_qos)
        self.ui_corrosion_area_accept_pub = self.create_publisher(Bool, '/ui/corrosion_area_accept_pub', 10)
        self.ui_corrosion_area_remove_pub = self.create_publisher(Image, '/ui/corrosion_area_remove_pub', image_qos)
        self.ui_connected_pub = self.create_publisher(Bool, '/ui/connected_pub', 10)
        self.ui_connected_pub_state = False
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        
        # MoveIt action client for collision-aware motion
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        self.move_group_ready = False

        # Z-axis jogging setup - uses TF + IK for proper Cartesian motion
        self.z_jog_active = False
        self.z_jog_direction = 0.0  # 0=stopped, +1=up, -1=down
        self.z_jog_increment = 0.01  # 10mm per step in base Z direction
        self.z_jog_pending_ik = False  # Track if IK call is in progress
        
        # XY joystick jogging setup - same IK-based approach
        self.xy_jog_active = False
        self.xy_jog_x = 0.0  # -1 to +1 (left to right)
        self.xy_jog_y = 0.0  # -1 to +1 (down to up)
        self.xy_jog_increment = 0.01  # 10mm per step
        self.xy_jog_pending_ik = False
        
        self.current_joint_states = None
        
        # TF buffer for getting current tool pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # IK service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # Timer for Z-axis continuous jogging at 5Hz
        self.z_jog_timer = self.create_timer(0.2, self.z_jog_step)
        
        # Timer for XY joystick jogging at 5Hz
        self.xy_jog_timer = self.create_timer(0.2, self.xy_jog_step)
        
        self.get_logger().info('Z-axis and XY jogging ready (Cartesian IK method)')

        self.corrosion_thresholding_pub = self.create_subscription(Image, '/corrosion/thresholding_pub', self.corrosion_thresholding_callback, image_qos)
        self.ROBODK_completion_notification = self.create_subscription(Bool, '/ROBODK/completion_notification_pub', self.ROBODK_completion_notification_callback, 10)
        color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw', qos_profile=image_qos)
        depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw', qos_profile=image_qos)
        sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        sync.registerCallback(self.image_match)
        
        self.last_Threshold_frame = None

        # Subscribe to /rosout to capture logs from ALL running nodes
        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            10
        )

        # Initialize UI components here (e.g., publishers/subscribers for UI commands)
        self.get_logger().info('User Interface Node Initialized')

    def rosout_callback(self, msg):
        """Handle log messages from /rosout (all nodes)."""
        level_map = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
        level = level_map.get(msg.level, 'UNKNOWN')
        node_name = msg.name
        text = msg.msg
        try:
            self.signal_emitter.log_signal.emit(f"[{level}] [{node_name}] {text}")
        except Exception:
            pass

    def _wrap_logger_for_gui(self):
        """Wrap logger methods to also emit messages to GUI log_signal."""
        try:
            orig_logger = super().get_logger()
            orig_info = orig_logger.info
            orig_warn = orig_logger.warn
            orig_error = orig_logger.error

            def _info(msg, *a, **k):
                orig_info(msg, *a, **k)
                try:
                    self.signal_emitter.log_signal.emit(f"[INFO] {msg}")
                except Exception:
                    pass

            def _warn(msg, *a, **k):
                orig_warn(msg, *a, **k)
                try:
                    self.signal_emitter.log_signal.emit(f"[WARN] {msg}")
                except Exception:
                    pass

            def _error(msg, *a, **k):
                orig_error(msg, *a, **k)
                try:
                    self.signal_emitter.log_signal.emit(f"[ERROR] {msg}")
                except Exception:
                    pass

            orig_logger.info = _info
            orig_logger.warn = _warn
            orig_logger.error = _error
        except Exception:
            pass

    def image_match(self, color_msg, depth_msg):
        try:
            # RealSense wrapper publishes RGB8, convert to BGR8 for OpenCV/display
            color_image = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, 3)
            color_image = cv.cvtColor(color_image, cv.COLOR_RGB2BGR)
            depth_image = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width)
            
            # Allocate corrosion_area_add and corrosion_area_remove only once on first image match
            if self.ui_instance is None:
                self.get_logger().warn('UI instance not ready yet, skipping frame')
                return
                
            if self.ui_instance.corrosion_area_add is None:
                h, w = color_image.shape[:2]
                self.ui_instance.corrosion_area_add = np.zeros((h, w), dtype=np.uint8)
                self.ui_instance.corrosion_area_remove = np.zeros((h, w), dtype=np.uint8)
                self.get_logger().info(f"Initialized corrosion_area_add and corrosion_area_remove with shape: {(h, w)} and {color_image.shape}")
        
            # Show color or depth based on camera_type
            if self.ui_instance.camerafeed[0] == 0 and self.ui_instance.camerafeed[1] == 0:
                self.signal_emitter.data_signal.emit(f"Color: {depth_image.shape[1]}x{depth_image.shape[0]}")
                self.signal_emitter.image_signal.emit(color_image)
                self.signal_emitter.painting_color_signal.emit("ffffff")  # White for color view
                if printlogger: self.get_logger().info('Switching to Color Camera')
            elif self.ui_instance.camerafeed[0] == 1 and self.ui_instance.camerafeed[1] == 1:
                depth_filtered = cv.medianBlur(depth_image, 5)
                depth_normalized = cv.normalize(depth_filtered, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
                depth_colormap = cv.applyColorMap(depth_normalized, cv.COLORMAP_JET)
                self.signal_emitter.data_signal.emit(f"Depth: {depth_image.shape[1]}x{depth_image.shape[0]}")
                self.signal_emitter.image_signal.emit(depth_colormap)
                self.signal_emitter.painting_color_signal.emit("B3B3B3")  # Gray for depth view
                if printlogger: self.get_logger().info('Showing Depth Camera')
            elif self.ui_instance.camerafeed[0] == 0 and self.ui_instance.camerafeed[1] == 1 and not self.ui_instance.is_painting:
                if self.last_Threshold_frame is not None:
                    self.signal_emitter.data_signal.emit(f"Thresholded: {self.last_Threshold_frame.shape[1]}x{self.last_Threshold_frame.shape[0]}")
                    self.signal_emitter.image_signal.emit(self.last_Threshold_frame)
                    self.signal_emitter.painting_color_signal.emit("ffffff")  # White for threshold view
                    if printlogger: self.get_logger().info('Switching to Color Camera')
        except Exception as e:
            self.get_logger().error(f'Error in image_match: {e}')

    def joint_state_callback(self, msg):
        """Store latest joint states"""
        self.current_joint_states = msg

    def z_jog_ik_response(self, future):
        """Handle IK service response"""
        self.z_jog_pending_ik = False
        try:
            response = future.result()
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                # Send joint trajectory with IK solution
                msg = JointTrajectory()
                msg.joint_names = list(response.solution.joint_state.name[:6])
                
                point = JointTrajectoryPoint()
                point.positions = list(response.solution.joint_state.position[:6])
                point.velocities = [0.0] * 6
                point.accelerations = [0.0] * 6
                point.time_from_start = Duration(sec=0, nanosec=180000000)  # 180ms
                
                msg.points = [point]
                self.joint_trajectory_pub.publish(msg)
            else:
                self.get_logger().warn(f'IK failed with error code: {response.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'IK response error: {e}')

    def xy_jog_ik_response(self, future):
        """Handle IK service response for XY jogging"""
        self.xy_jog_pending_ik = False
        try:
            response = future.result()
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                # Send joint trajectory with IK solution
                msg = JointTrajectory()
                msg.joint_names = list(response.solution.joint_state.name[:6])
                
                point = JointTrajectoryPoint()
                point.positions = list(response.solution.joint_state.position[:6])
                point.velocities = [0.0] * 6
                point.accelerations = [0.0] * 6
                point.time_from_start = Duration(sec=0, nanosec=180000000)  # 180ms
                
                msg.points = [point]
                self.joint_trajectory_pub.publish(msg)
            else:
                if printlogger:
                    self.get_logger().warn(f'XY IK failed with error code: {response.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'XY IK response error: {e}')

    def z_jog_step(self):
        """Send incremental Cartesian Z movement via IK + joint trajectory"""
        if self.z_jog_direction == 0.0:
            if self.z_jog_active:
                self.z_jog_active = False
                self.get_logger().info('Z-jogging stopped')
            return
        
        if self.current_joint_states is None:
            self.get_logger().warn('No joint states yet')
            return
            
        if self.z_jog_pending_ik:
            return  # Skip if previous IK call still pending
        
        if not self.z_jog_active:
            self.z_jog_active = True
            self.get_logger().info(f'Z-jogging active: {"UP" if self.z_jog_direction > 0 else "DOWN"}')
        
        try:
            # Get current tool pose from TF
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Create target pose with Z offset in base_link frame
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'base_link'
            target_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Copy current position and add Z offset
            target_pose.pose.position.x = transform.transform.translation.x
            target_pose.pose.position.y = transform.transform.translation.y
            target_pose.pose.position.z = transform.transform.translation.z + (self.z_jog_increment * self.z_jog_direction)
            
            # Keep orientation constant
            target_pose.pose.orientation.x = transform.transform.rotation.x
            target_pose.pose.orientation.y = transform.transform.rotation.y
            target_pose.pose.orientation.z = transform.transform.rotation.z
            target_pose.pose.orientation.w = transform.transform.rotation.w
            
            # Call IK service
            if not self.ik_client.service_is_ready():
                self.get_logger().warn('IK service not ready', throttle_duration_sec=1.0)
                return
                
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = 'ur_manipulator'
            ik_request.ik_request.pose_stamped = target_pose
            ik_request.ik_request.avoid_collisions = False
            ik_request.ik_request.timeout = Duration(sec=0, nanosec=50000000)
            
            # Set current joint state as seed
            ik_request.ik_request.robot_state.joint_state = self.current_joint_states
            
            # Call IK asynchronously
            self.z_jog_pending_ik = True
            future = self.ik_client.call_async(ik_request)
            future.add_done_callback(self.z_jog_ik_response)
            
        except Exception as e:
            self.get_logger().error(f'Z-jog error: {e}')

    def xy_jog_step(self):
        # Check if joystick is centered (no movement)
        if abs(self.xy_jog_x) < 0.05 and abs(self.xy_jog_y) < 0.05:
            if self.xy_jog_active:
                self.xy_jog_active = False
                self.get_logger().info('XY jogging stopped')
            return
        
        if self.current_joint_states is None:
            return
            
        if self.xy_jog_pending_ik:
            return  # Skip if previous IK call still pending
        
        if not self.xy_jog_active:
            self.xy_jog_active = True
            self.get_logger().info('XY jogging active')
        
        try:
            # Get current tool pose from TF
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Create target pose with XY offset in base_link frame
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'base_link'
            target_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Apply X and Y offsets based on joystick position
            target_pose.pose.position.x = transform.transform.translation.x + (self.xy_jog_increment * self.xy_jog_x)
            target_pose.pose.position.y = transform.transform.translation.y + (self.xy_jog_increment * self.xy_jog_y)
            target_pose.pose.position.z = transform.transform.translation.z  # Keep Z constant
            
            # Keep orientation constant
            target_pose.pose.orientation.x = transform.transform.rotation.x
            target_pose.pose.orientation.y = transform.transform.rotation.y
            target_pose.pose.orientation.z = transform.transform.rotation.z
            target_pose.pose.orientation.w = transform.transform.rotation.w
            
            # Call IK service
            if not self.ik_client.service_is_ready():
                return
                
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = 'ur_manipulator'
            ik_request.ik_request.pose_stamped = target_pose
            ik_request.ik_request.avoid_collisions = False
            ik_request.ik_request.timeout = Duration(sec=0, nanosec=50000000)
            
            # Set current joint state as seed
            ik_request.ik_request.robot_state.joint_state = self.current_joint_states
            
            # Call IK asynchronously
            self.xy_jog_pending_ik = True
            future = self.ik_client.call_async(ik_request)
            future.add_done_callback(self.xy_jog_ik_response)
            
        except Exception as e:
            if printlogger:
                self.get_logger().error(f'XY-jog error: {e}')

    def corrosion_thresholding_callback(self, msg):
        if self.ui_connected_pub_state ==False:
            self.ui_connected_pub_state = True
            connected_msg = Bool()
            connected_msg.data = True
            self.ui_connected_pub.publish(connected_msg)
            self.get_logger().info('Published UI connected state as True')

        corrosion_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self.last_Threshold_frame = corrosion_image
        if printlogger:
            self.get_logger().info('=== Corrosion thresholding callback CALLED ===')
            self.get_logger().info(f'Saved threshold frame with shape: {corrosion_image.shape}')            

    def accept_corrosion_area(self, accept: bool):
        # Logic to accept or reject corrosion area
        accept_msg = Bool()
        accept_msg.data = True
        self.ui_corrosion_area_accept_pub.publish(accept_msg)
        if printlogger: self.get_logger().info(f'Accepting corrosion area: {accept}')

    def ROBODK_completion_notification_callback(self, msg):
        if msg.data == True:
            self.currently_running = False
            if printlogger: self.get_logger().info('ROBODK has completed the path, ready for new corrosion area')
        elif msg.data == False:
            self.currently_running = True
            if printlogger: self.get_logger().info('ROBODK has started the path')

class UserInterface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.camerafeed = [0,0]
        self.tabindex = 0  # Track current tab index
        self.pen_size_and_type = [0,1]  # [size, type]
        
        # Create single-channel image variables (will be dynamically sized)
        self.undo_add_stack = []
        self.undo_remove_stack = []
        self.corrosion_area_add = None
        self.corrosion_area_remove = None
        self.last_frame = None
        self.is_painting = False  # Track if currently painting

        
        self.ui.RUN_1.clicked.connect(self.run_robot)
        self.ui.RUN_2.clicked.connect(self.run_robot)
        self.ui.Undo.clicked.connect(self.undo_action)
        self.ui.Eraser.clicked.connect(self.erase_area)
        self.ui.Reset.clicked.connect(self.reset_vision)
        self.ui.Terminate.clicked.connect(self.terminate)
        self.ui.Home_Position.clicked.connect(self.home_position)
        self.ui.Emergency_Stop.clicked.connect(self.emergency_stop)
        
        # Z-axis jogging buttons - use pressed/released for continuous motion
        self.ui.pushButton_Z_up.pressed.connect(self.z_up_pressed)
        self.ui.pushButton_Z_up.released.connect(self.z_released)
        self.ui.pushButton_Z_down.pressed.connect(self.z_down_pressed)
        self.ui.pushButton_Z_down.released.connect(self.z_released)
        
        self.ui.videoLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.ui.Vision_State.clicked.connect(lambda: self.feed_toggle(1)) # Turn on and off threshold view on tab 1
        self.ui.Small_Pen.clicked.connect(lambda: self.set_custom_pen(0))
        self.ui.Medium_Pen.clicked.connect(lambda: self.set_custom_pen(1))
        self.ui.Large_Pen.clicked.connect(lambda: self.set_custom_pen(2))
        self.ui.Switch_Camera_Type.clicked.connect(lambda: self.feed_toggle(0))#switch between threshold and depth
        self.ui.infoButton.toggled.connect(self.ui.stackedWidget_Info.setVisible)  # Connect info button
        self.ui.infoButton.setEnabled(True)  # Make sure it's enabled
        self.ui.infoButton.setFocusPolicy(Qt.FocusPolicy.StrongFocus)  # Ensure focus can reach it

        self.ui.tabWidget.currentChanged.connect(lambda index: self.tab_difference(index))
        self.signal_emitter = RosSignalEmitter()
        self.signal_emitter.data_signal.connect(self.ui.videoLabel.setText)
        self.signal_emitter.image_signal.connect(self.update_video_frame)
        self.signal_emitter.painting_color_signal.connect(self.update_painting_button_colors)

        # Replace sysinfoLabel with a scrollable QTextEdit for ROS logs
        # Remove the label and add QTextEdit in the same layout slot
        self.ui.sysinfoLabel.hide()
        self.log_widget = QTextEdit()
        self.log_widget.setReadOnly(True)
        self.log_widget.setMaximumHeight(80)  # Keep it small to not affect other UI
        self.ui.horizontalLayout_5.addWidget(self.log_widget)
        self.signal_emitter.log_signal.connect(self.append_log)

        self.ros_node = UserInterfaceNode(self.signal_emitter, self)  # Pass self for state access
    
        self.ui.videoLabel.clicked.connect(self.on_image_clicked)
        self.ui.videoLabel.dragged.connect(self.on_image_dragged)
        self.ui.videoLabel.released.connect(self.on_image_released)
        
        # Connect joystick signals
        self.ui.Joystick.touched.connect(self.on_joystick_touched)
        self.ui.Joystick.moved.connect(self.on_joystick_moved)
        self.ui.Joystick.released.connect(self.on_joystick_released)
        
        # Hide statusLabel
        self.ui.statusLabel.hide()
        self.ui.stackedWidget_Info.hide()
        self.ui.frame_5.hide()



        self.ui.Info_Movement_Label.setWordWrap(True)  # Enable auto word wrap
        self.ui.Info_Movement_Label.setText("<b>Emergency Stop</b><br>"
                                            "Perform an emergency stop<br>"
                                            "<b>Home Position</b><br>"
                                            "Home robot arm<br>"
                                            "<b>Vision State</b><br>"
                                            "Change between image view and detection view<br>"
                                            "<b>Run</b><br>"
                                            "Approve/begin operation<br>"
                                            "<b>Joystick and Z+/-</b><br>"
                                            "Adjust robot arm position<br>"
                                            "<b>Terminate</b><br>"
                                            "Stop current operation")

        self.ui.Info_Vision_Label.setWordWrap(True)  # Enable auto word wrap
        self.ui.Info_Vision_Label.setText("<b>Emergency Stop</b><br>"
                                            "Perform an emergency stop<br>"
                                            "<b> Switch Camera Type</b><br>"
                                            "Switch detection/depth camera view<br>"
                                            "<b>Painting features</b><br>"
                                            "<b>- Reset:</b> painted adjustments on image<br>"
                                            "<b>- Undo:</b> last adjustment on image<br>"
                                            "<b>- Erase:</b> Toggle erase mode<br>"      
                                            "<b>- Pen size:</b> adjust size of pen<br>"                                                                                                                                                                           
                                            "<b>Run</b><br>"
                                            "Approve/begin operation<br>")

        self.ui.Info_System_Label.setWordWrap(True)  # Enable auto word wrap
        self.ui.Info_System_Label.setText("<b>Emergency Stop</b><br>"
                                            "Perform an emergency stop<br>"
                                            "<b>Joystick and Z+/-</b><br>"
                                            "Adjust robot arm position<br>"
                                            "<b>Terminate</b><br>"
                                            "Stop current operation")


    def customize_tabs(self):
        tabbar = self.ui.tabWidget.tabBar()
        tabbar.setExpanding(True)
        tabbar.setTabsClosable(False)
        tab_width = self.ui.tabWidget.width() // 3
        self.ui.tabWidget.setStyleSheet(f"QTabBar::tab {{width: {tab_width}px; min-width: {tab_width // self.ui.tabWidget.count()}px;}}")

    def append_log(self, text: str):
        """Append a log line to the GUI log widget with timestamp."""
        try:
            from datetime import datetime
            ts = datetime.now().strftime('%H:%M:%S')
            self.log_widget.append(f"[{ts}] {text}")
        except Exception:
            pass

    def update_painting_button_colors(self, color):
        """Update painting button colors - called from Qt thread via signal"""
        self.ui.Reset.setStyleSheet(f"background-color: #{color};")
        self.ui.Undo.setStyleSheet(f"background-color: #{color};")
        self.ui.Small_Pen.setStyleSheet(f"background-color: #{color};")
        self.ui.Medium_Pen.setStyleSheet(f"background-color: #{color};")
        self.ui.Large_Pen.setStyleSheet(f"background-color: #{color};")
        # Preserve Eraser active state - only update if not in erase mode
        if self.pen_size_and_type[1] == 1:  # Not in erase mode
            self.ui.Eraser.setStyleSheet(f"background-color: #{color};")
        # else: keep the active (gray) color

    def z_up_pressed(self):
        """Start continuous Z+ motion when button is pressed"""
        self.ros_node.z_jog_direction = 1.0
        self.ros_node.get_logger().info('⬆ Z+ jogging started')
    
    def z_down_pressed(self):
        """Start continuous Z- motion when button is pressed"""
        self.ros_node.z_jog_direction = -1.0
        self.ros_node.get_logger().info('⬇ Z- jogging started')
    
    def z_released(self):
        """Stop Z-axis motion when button is released"""
        self.ros_node.z_jog_direction = 0.0
        self.ros_node.get_logger().info('⏹ Z jogging stopped')

    def emergency_stop(self):
        msg = Bool()
        msg.data = True
        self.ros_node.ui_emergency_stop_pub.publish(msg)
        self.joystick_terminate_change_page(False)
        if printlogger: self.ros_node.get_logger().info('Emergency Stop pressed')

    def home_position(self):
        import math
        
        # Check if MoveIt action server is available
        if not self.ros_node.move_group_client.wait_for_server(timeout_sec=1.0):
            self.ros_node.get_logger().error('MoveIt /move_action not available! Cannot home safely.')
            QMessageBox.warning(self, 'Homing Failed', 
                'MoveIt is not available. Cannot perform collision-checked homing.\n'
                'Make sure MoveIt is running (launch_ur_moveit.sh).')
            return
        
        # Home position: 90, -90, 90, -90, -90, 0 degrees
        home_angles = [math.radians(90), math.radians(-90), math.radians(90),
                       math.radians(-90), math.radians(-90), math.radians(0)]
        
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Create MoveGroup goal with joint constraints (minimal config like working scripts)
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        
        # Create joint constraints for home position
        goal_constraints = Constraints()
        for name, angle in zip(joint_names, home_angles):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = angle
            jc.tolerance_above = 0.01  # ~0.6 degrees tolerance
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        self.ros_node.get_logger().info('Sending home position goal to MoveIt (collision-aware)...')
        
        # Send goal asynchronously
        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.ros_node.get_logger().error('Home position goal was rejected by MoveIt')
                return
            
            self.ros_node.get_logger().info('Home position goal accepted, planning and executing...')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_callback)
        
        def result_callback(future):
            result = future.result().result
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.ros_node.get_logger().info('✓ Robot successfully moved to home position!')
            else:
                error_meanings = {
                    -1: 'FAILURE',
                    -2: 'PLANNING_FAILED', 
                    -26: 'FRAME_TRANSFORM_FAILURE',
                    -31: 'NO_IK_SOLUTION',
                    -13: 'GOAL_IN_COLLISION',
                    -12: 'INVALID_MOTION_PLAN'
                }
                error_name = error_meanings.get(result.error_code.val, f'ERROR_{result.error_code.val}')
                self.ros_node.get_logger().error(f'Homing failed: {error_name}')
        
        send_goal_future = self.ros_node.move_group_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(goal_response_callback)
    
    def run_robot(self):
        reply = QMessageBox.question(
            self, 
            'Confirm Action', 
            'Are you sure you want to confirm the corrosion area and start process?',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No  # Default to No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            msg = Bool()
            msg.data = True
            self.ui.stackedWidget.setCurrentIndex(0)
            self.joystick_terminate_change_page(True)
            self.ros_node.ui_corrosion_area_accept_pub.publish(msg)
            if printlogger: self.ros_node.get_logger().info('Run Robot pressed')
        else:
            if printlogger: self.ros_node.get_logger().info('Run robot pressed cancelled')

        msg = Bool()
        msg.data = True
        self.ui.stackedWidget.setCurrentIndex(0)
        self.joystick_terminate_change_page(True)
        self.ros_node.ui_corrosion_area_accept_pub.publish(msg)
        if printlogger: self.ros_node.get_logger().info('Run Robot pressed')
    
    def joystick_terminate_change_page(self, state):
        if state:
            self.ui.stackedWidget.setCurrentIndex(0)
            if printlogger: self.ros_node.get_logger().info(f'Swithing to terminate page')
        elif not state:
            self.ui.stackedWidget.setCurrentIndex(1)
            if printlogger: self.ros_node.get_logger().info(f'Swithing to joystick page')

    def terminate(self):
        reply = QMessageBox.question(
            self, 
            'Confirm Terminate', 
            'Are you sure you want to terminate?',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No  # Default to No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            self.joystick_terminate_change_page(False)
            msg = Bool()
            msg.data = True
            self.ros_node.ui_terminate_pub.publish(msg)
            if printlogger: self.ros_node.get_logger().info('Terminate confirmed')
        else:
            if printlogger: self.ros_node.get_logger().info('Terminate cancelled')

    def feed_toggle(self, index):
        self.camerafeed[index] = 1 - self.camerafeed[index]  # Toggle between 0 and 1
        if printlogger: self.ros_node.get_logger().info(f'Toggling camera feed {index} to {self.camerafeed[index]}')

    def reset_vision(self):
        if not self.camerafeed[0] == 1:
            self.ros_node.get_logger().info('Resetting vision areas')
            if self.corrosion_area_add is not None:
                h, w = self.corrosion_area_add.shape
                self.corrosion_area_add = np.zeros((h, w), dtype=np.uint8)
                self.corrosion_area_remove = np.zeros((h, w), dtype=np.uint8)
                self.undo_add_stack.clear() 
                self.undo_remove_stack.clear()
                self.ros_node.ui_corrosion_area_remove_pub.publish(self.numpy_to_image_msg(self.corrosion_area_remove, 'mono8'))
                self.ros_node.ui_corrosion_area_add_pub.publish(self.numpy_to_image_msg(self.corrosion_area_add, 'mono8'))
            else:
                self.ros_node.get_logger().info('Corrosion areas not initialized yet, skipping reset')

            if printlogger: self.ros_node.get_logger().info('Resetting Vision')

    def undo_action(self):
        if not self.camerafeed[0] == 1:
            if len(self.undo_add_stack) > 0:
                self.corrosion_area_add = self.undo_add_stack.pop()
                self.corrosion_area_remove = self.undo_remove_stack.pop()
                
                # Publish updated masks
                add_msg = self.numpy_to_image_msg(self.corrosion_area_add, 'mono8')
                remove_msg = self.numpy_to_image_msg(self.corrosion_area_remove, 'mono8')
                self.ros_node.ui_corrosion_area_add_pub.publish(add_msg)
                self.ros_node.ui_corrosion_area_remove_pub.publish(remove_msg)
                
                if printlogger:
                    self.ros_node.get_logger().info(f'Undo applied (stack size: {len(self.undo_add_stack)})')
            else:
                self.ros_node.get_logger().info('Nothing to undo')

    def erase_area(self):
        if not self.camerafeed[0] == 1:
            self.pen_size_and_type[1] = 1 - self.pen_size_and_type[1]
            if self.pen_size_and_type[1] == 0:
                self.ui.Eraser.setStyleSheet("background-color: #999999;")
            else:
                self.ui.Eraser.setStyleSheet("background-color: #ffffff;")

            if printlogger: self.ros_node.get_logger().info(f'Erase area requested{"" if self.pen_size_and_type[1] == 0 else " (Eraser Mode)"}')

    def set_custom_pen(self, size):
        self.pen_size_and_type[0] = size
        if printlogger: self.ros_node.get_logger().info(f'Set pen size to {size}')

    def tab_difference(self, index):
        if printlogger: self.ros_node.get_logger().info(f'Tab changed to {index}')
        if index == 0:
            self.camerafeed = [0,0]
            self.tabindex = index
            self.ui.stackedWidget_Info.setCurrentIndex(0) 
        elif index == 1:
            self.camerafeed = [0,1]
            self.tabindex = index
            self.ui.stackedWidget_Info.setCurrentIndex(1) 

            if self.ros_node.last_Threshold_frame is not None:
                self.signal_emitter.data_signal.emit(f"Thresholded: {self.ros_node.last_Threshold_frame.shape[1]}x{self.ros_node.last_Threshold_frame.shape[0]}")
                self.signal_emitter.image_signal.emit(self.ros_node.last_Threshold_frame)
                if printlogger:
                    self.ros_node.get_logger().info('Displayed cached threshold frame on tab switch')
            else:
                if printlogger:
                    self.ros_node.get_logger().info('No cached threshold frame available yet')
        elif index == 2:
            self.camerafeed = [0,0]
            self.ui.stackedWidget_Info.setCurrentIndex(2)  # Show System info
            pass  # System Information tab

    def on_joystick_touched(self):
        if printlogger: self.ros_node.get_logger().info('Joystick touched!')
        
        #Check if there are any markings to be lost
        if self.corrosion_area_add is not None and self.corrosion_area_remove is not None:
            has_add = np.any(self.corrosion_area_add > 0)
            has_remove = np.any(self.corrosion_area_remove > 0)
            
            if has_add or has_remove:
                # Show confirmation dialog
                reply = QMessageBox.question(
                    self,
                    'Confirm movement', 
                    'Are you sure you want make a movement?\n This will result in all corrosion area markings being lost.',
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No
                )
                
                if reply == QMessageBox.StandardButton.Yes:
                    self.reset_vision()
                    if printlogger: self.ros_node.get_logger().info('Adjustments reset by joystick touch')
    
    def on_joystick_moved(self, direction_tuple):
        from user_interface.joystick import Direction
        
        direction, distance = direction_tuple
        
        # Convert direction enum to XY coordinates (-1 to +1)
        # X: negative=left, positive=right
        # Y: negative=down, positive=up
        if direction == Direction.Right:
            self.ros_node.xy_jog_x = distance
            self.ros_node.xy_jog_y = 0.0
        elif direction == Direction.Left:
            self.ros_node.xy_jog_x = -distance
            self.ros_node.xy_jog_y = 0.0
        elif direction == Direction.Up:
            self.ros_node.xy_jog_x = 0.0
            self.ros_node.xy_jog_y = distance
        elif direction == Direction.Down:
            self.ros_node.xy_jog_x = 0.0
            self.ros_node.xy_jog_y = -distance
        
        if printlogger:
            self.ros_node.get_logger().info(f'Joystick: {direction.name} dist={distance:.2f} -> X={self.ros_node.xy_jog_x:.2f} Y={self.ros_node.xy_jog_y:.2f}')
    
    def on_joystick_released(self):
        """Stop XY jogging when joystick is released"""
        self.ros_node.xy_jog_x = 0.0
        self.ros_node.xy_jog_y = 0.0
        if printlogger:
            self.ros_node.get_logger().info('Joystick released - XY jogging stopped')
    
    def update_video_frame(self, img):
        """Convert numpy BGR array to QPixmap and display in videoLabel"""
        if img is None:
            return
        self.last_frame = img.copy()

        if len(img.shape) == 3:
            h, w, ch = img.shape
            bytes_per_line = ch * w
            qimage = QImage(img.data.tobytes(), w, h, bytes_per_line, QImage.Format.Format_BGR888).copy()
        else:
            h, w = img.shape
            bytes_per_line = w
            qimage = QImage(img.data.tobytes(), w, h, bytes_per_line, QImage.Format.Format_Grayscale8).copy()
        
        pixmap = QPixmap.fromImage(qimage)
        scaled = pixmap.scaled(self.ui.videoLabel.size(), Qt.AspectRatioMode.KeepAspectRatio,
                               Qt.TransformationMode.SmoothTransformation)
        self.ui.videoLabel.setPixmap(scaled)
        
        # Set the actual image dimensions for coordinate mapping
        self.ui.videoLabel.set_image_dimensions(w, h)

    def place_kernel(self, img, kernel, x, y):
        """Place kernel centered at (x,y) in img using loops."""
        kernel_height, kernel_width = kernel.shape
        img_height, img_width = img.shape

        kernel_mid_y, kernel_mid_x = kernel_height // 2, kernel_width // 2

        for kernel_pos_y in range(kernel_height):
            for kernel_pos_x in range(kernel_width):
                # Target position in image
                img_y = y - kernel_mid_y + kernel_pos_y
                img_x = x - kernel_mid_x + kernel_pos_x

                # Skip if out of bounds
                if 0 <= img_y < img_height and 0 <= img_x < img_width:
                    img[img_y, img_x] = kernel[kernel_pos_y, kernel_pos_x]
                    self.last_frame[img_y, img_x, 0] = kernel[kernel_pos_y, kernel_pos_x]
        self.update_video_frame(self.last_frame)


    def numpy_to_image_msg(self, img, encoding):
        #Might be possible to shorten with the use of CvBridge
        msg = Image()
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = img.shape[1] * img.itemsize * (3 if len(img.shape) == 3 else 1)
        msg.data = img.tobytes()
        return msg

    def on_image_dragged(self, x, y, button):
        """Handle image drag events"""
        if self.camerafeed[1] == 1 and self.camerafeed[0] == 0 and self.tabindex == 1 and self.pen_size_and_type[0] in [0, 1, 2] and (self.pen_size_and_type[0] and self.pen_size_and_type[1]) is not None:
            if printlogger: self.ros_node.get_logger().info(f'Pen size: {self.pen_size_and_type[0]}, Pen type: {self.pen_size_and_type[1]}')
            self.pen_kernel_sizes = [   np.ones((10, 10), np.uint8) * 255,   
                                        np.ones((25, 25), np.uint8) * 255,
                                        np.ones((50, 50), np.uint8) * 255]            
            if self.pen_size_and_type[1]==1:
                if printlogger: self.ros_node.get_logger().info("Using add kernels")
                self.place_kernel(self.corrosion_area_add, self.pen_kernel_sizes[self.pen_size_and_type[0]], x, y)

            elif self.pen_size_and_type[1]==0:
                if printlogger: self.ros_node.get_logger().info("Using remove kernels")
                self.place_kernel(self.corrosion_area_remove, self.pen_kernel_sizes[self.pen_size_and_type[0]], x, y)
        pass
    
    def save_undo_state(self):
        """Save current state before making changes"""
        if self.corrosion_area_add is not None:
            # Save current versions of the corrosion areas to the undo stacks
            self.undo_add_stack.append(self.corrosion_area_add.copy())
            self.undo_remove_stack.append(self.corrosion_area_remove.copy())
            
            # Limit stack size so memory does not grow indefinitely
            max_undo_levels = 20
            if len(self.undo_add_stack) > max_undo_levels:
                self.undo_add_stack.pop(0)  
                self.undo_remove_stack.pop(0)
                
            if printlogger:
                self.ros_node.get_logger().info(f'Saved undo state (stack size: {len(self.undo_add_stack)})')
    

    def on_image_clicked(self, x, y, button):
        if self.camerafeed[1] == 1 and self.camerafeed[0] == 0 and self.tabindex == 1:
            # Save state BEFORE first paint
            self.save_undo_state()
            self.is_painting = True  # Start painting mode
            
        if printlogger:
            self.ros_node.get_logger().info(f'Click at ({x}, {y}), saved undo state')

    def on_image_released(self, x, y, button):
        """Handle mouse button release - fires when drag ends"""
        if printlogger:
            self.ros_node.get_logger().info(f'Released at ({x}, {y}), button={button}')
        
        # This fires AFTER dragging stops
        if self.camerafeed[1] == 1 and self.camerafeed[0] == 0 and self.tabindex == 1:
            self.is_painting = False  # End painting mode - allow ROS updates again
            
            # Publish final state to ROS for processing
            msgadd = Image()
            msgadd = self.numpy_to_image_msg(self.corrosion_area_add, 'mono8')
            self.ros_node.ui_corrosion_area_add_pub.publish(msgadd)

            msgremove = Image()
            msgremove = self.numpy_to_image_msg(self.corrosion_area_remove, 'mono8')
            self.ros_node.ui_corrosion_area_remove_pub.publish(msgremove)

            self.ros_node.get_logger().info('Stroke completed, published masks')

    def closeEvent(self, event):
        """Called when window closes - cleanup before shutdown"""
        self.cleanup()
        event.accept()
    
    def cleanup(self):
        """Cleanup function to run before topics disable"""
        self.ros_node.get_logger().info('Cleaning up - publishing final state')
        # Publish terminate signal
        msg = Bool()
        msg.data = False
        self.ros_node.ui_connected_pub.publish(msg)
        self.reset_vision()

        # Add any other cleanup here



def main():
    import signal
    
    rclpy.init()
    app = QApplication(sys.argv)
    window = UserInterface()
    
    # Handle Ctrl-C
    def signal_handler(sig, frame):
        window.cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    window.show()
    window.customize_tabs()
    ros_thread = threading.Thread(target=lambda: rclpy.spin(window.ros_node), daemon=True)
    ros_thread.start()
    
    exit_code = app.exec()
    
    # Cleanup when app exits
    window.cleanup()
    window.ros_node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()


