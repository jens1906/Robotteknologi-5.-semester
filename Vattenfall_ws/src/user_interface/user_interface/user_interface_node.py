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
from sensor_msgs.msg import Image
from PyQt6.QtGui import QImage, QPixmap, QFont
from user_interface.GUI import Ui_MainWindow
from PyQt6.QtCore import pyqtSignal, QObject, Qt
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QMessageBox
from user_interface.joystick import Joystick
import signal

Test = True
showImages = True
printlogger = False



class RosSignalEmitter(QObject):
    data_signal = pyqtSignal(str)           
    image_signal = pyqtSignal(object)       

class UserInterfaceNode(Node):
    def __init__(self, signal_emitter, ui_instance=None):
        super().__init__('user_interface')
        self.signal_emitter = signal_emitter
        self.ui_instance = ui_instance
        
        # QoS profile for image topics (best effort for network transmission)
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
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


        self.corrosion_thresholding_pub = self.create_subscription(Image, '/corrosion/thresholding_pub', self.corrosion_thresholding_callback, image_qos)
        self.ROBODK_completion_notification = self.create_subscription(Bool, '/ROBODK/completion_notification_pub', self.ROBODK_completion_notification_callback, 10)
        color_sub = message_filters.Subscriber(self, Image, '/realsense/camera_color_pub', qos_profile=image_qos)
        depth_sub = message_filters.Subscriber(self, Image, '/realsense/camera_depth_pub', qos_profile=image_qos)
        sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        sync.registerCallback(self.image_match)
        
        self.last_Threshold_frame = None

        # Initialize UI components here (e.g., publishers/subscribers for UI commands)
        self.get_logger().info('User Interface Node Initialized')

    def image_match(self, color_msg, depth_msg):
        color_image = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, 3)
        depth_image = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width)
        

        # Allocate corrosion_area_add and corrosion_area_remove only once on first image match
        if self.ui_instance.corrosion_area_add is None:
            h, w = color_image.shape[:2]
            self.ui_instance.corrosion_area_add = np.zeros((h, w), dtype=np.uint8)
            self.ui_instance.corrosion_area_remove = np.zeros((h, w), dtype=np.uint8)
            self.get_logger().info(f"Initialized corrosion_area_add and corrosion_area_remove with shape: {(h, w)} and {color_image.shape}")
        # Show color or depth based on camera_type
        if self.ui_instance.camerafeed[0] == 0 and self.ui_instance.camerafeed[1] == 0:
            self.signal_emitter.data_signal.emit(f"Color: {depth_image.shape[1]}x{depth_image.shape[0]}")
            self.signal_emitter.image_signal.emit(color_image)
            self.ui_instance.ui.Reset.setStyleSheet("background-color: #ffffff;")
            self.ui_instance.ui.Eraser.setStyleSheet("background-color: #ffffff;")
            self.ui_instance.ui.Undo.setStyleSheet("background-color: #ffffff;")
            self.ui_instance.ui.Small_Pen.setStyleSheet("background-color: #ffffff;")
            self.ui_instance.ui.Medium_Pen.setStyleSheet("background-color: #ffffff;")
            self.ui_instance.ui.Large_Pen.setStyleSheet("background-color: #ffffff;")
            if printlogger: self.ros_node.get_logger().info('Switching to Color Camera')
        elif self.ui_instance.camerafeed[0] == 1 and self.ui_instance.camerafeed[1] == 1:
            self.signal_emitter.data_signal.emit(f"Depth: {depth_image.shape[1]}x{depth_image.shape[0]}")
            self.signal_emitter.image_signal.emit(depth_image)
            self.ui_instance.ui.Reset.setStyleSheet("background-color: #B3B3B3;")
            self.ui_instance.ui.Eraser.setStyleSheet("background-color: #B3B3B3;")
            self.ui_instance.ui.Undo.setStyleSheet("background-color: #B3B3B3;")
            self.ui_instance.ui.Small_Pen.setStyleSheet("background-color: #B3B3B3;")
            self.ui_instance.ui.Medium_Pen.setStyleSheet("background-color: #B3B3B3;")
            self.ui_instance.ui.Large_Pen.setStyleSheet("background-color: #B3B3B3;")
            if printlogger: self.ros_node.get_logger().info('Switching to Depth Camera')
        elif self.ui_instance.camerafeed[0] == 0 and self.ui_instance.camerafeed[1] == 1 and not self.ui_instance.is_painting:            
            self.signal_emitter.data_signal.emit(f"Thresholded: {self.last_Threshold_frame.shape[1]}x{self.last_Threshold_frame.shape[0]}")
            self.signal_emitter.image_signal.emit(self.last_Threshold_frame)
            self.ui_instance.ui.Reset.setStyleSheet("background-color: #ffffff;")
            self.ui_instance.ui.Eraser.setStyleSheet("background-color: #ffffff;")
            self.ui_instance.ui.Undo.setStyleSheet("background-color: #ffffff;")
            self.ui_instance.ui.Small_Pen.setStyleSheet("background-color: #ffffff;")
            self.ui_instance.ui.Medium_Pen.setStyleSheet("background-color: #ffffff;")
            self.ui_instance.ui.Large_Pen.setStyleSheet("background-color: #ffffff;")
            if printlogger: self.ros_node.get_logger().info('Switching to Color Camera')


    def corrosion_thresholding_callback(self, msg):
        self.get_logger().info('=== Corrosion thresholding callback CALLED ===')
        if self.ui_connected_pub_state ==False:
            self.ui_connected_pub_state = True
            connected_msg = Bool()
            connected_msg.data = True
            self.ui_connected_pub.publish(connected_msg)
            self.get_logger().info('Published UI connected state as True')

        corrosion_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self.last_Threshold_frame = corrosion_image
        self.get_logger().info(f'Saved threshold frame with shape: {corrosion_image.shape}')

            
    def ROBODK_completion_notification_callback(self, msg):
        if msg.data == True:
            self.currently_running = False
            if printlogger: self.get_logger().info('ROBODK has completed the path, ready for new corrosion area')
        elif msg.data == False:
            self.currently_running = True
            if printlogger: self.get_logger().info('ROBODK has started the path')

    def accept_corrosion_area(self, accept: bool):
        # Logic to accept or reject corrosion area
        accept_msg = Bool()
        accept_msg.data = True
        self.ui_corrosion_area_accept_pub.publish(accept_msg)
        if printlogger: self.get_logger().info(f'Accepting corrosion area: {accept}')

    def erase_corrosion_area(self):
        # Logic to erase corrosion area
        erase_msg = Image()
        self.ui_corrosion_area_remove_pub.publish(erase_msg)
        if printlogger: self.get_logger().info('Erasing corrosion area')

    def add_corrosion_area(self):
        # Logic to add corrosion area
        add_msg = Image()
        self.ui_corrosion_area_add_pub.publish(add_msg)
        if printlogger: self.get_logger().info('Adding corrosion area')

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
        self.ui.videoLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.ui.Vision_State.clicked.connect(self.toggle_vision_state) # Turn on and off threshold view on tab 1
        self.ui.Small_Pen.clicked.connect(lambda: self.set_custom_pen(0))
        self.ui.Medium_Pen.clicked.connect(lambda: self.set_custom_pen(1))
        self.ui.Large_Pen.clicked.connect(lambda: self.set_custom_pen(2))
        self.ui.Switch_Camera_Type.clicked.connect(self.switch_camera_type)#switch between threshold and depth
        self.ui.infoButton.toggled.connect(self.toggle_info_panel)  # Connect info button
        self.ui.infoButton.setEnabled(True)  # Make sure it's enabled
        self.ui.infoButton.setFocusPolicy(Qt.FocusPolicy.StrongFocus)  # Ensure focus can reach it

        self.ui.tabWidget.currentChanged.connect(lambda index: self.tab_difference(index))
        self.signal_emitter = RosSignalEmitter()
        self.signal_emitter.data_signal.connect(self.on_data)
        self.signal_emitter.image_signal.connect(self.update_video_frame)
        self.ros_node = UserInterfaceNode(self.signal_emitter, self)  # Pass self for state access
    
        self.ui.videoLabel.clicked.connect(self.on_image_clicked)
        self.ui.videoLabel.dragged.connect(self.on_image_dragged)
        self.ui.videoLabel.released.connect(self.on_image_released)
        
        # Connect joystick signals
        self.ui.Joystick.touched.connect(self.on_joystick_touched)
        self.ui.Joystick.released.connect(self.on_joystick_released)
        self.ui.Joystick.moved.connect(self.on_joystick_moved)
        
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
        
        # Calculate equal width for each tab based on actual widget width
        tab_count = self.ui.tabWidget.count()
        if tab_count > 0:
            tabbar.setTabsClosable(False)
            
            # Get the actual width of the tab widget
            tab_widget_width = self.ui.tabWidget.width()//3
            width_per_tab = tab_widget_width // tab_count
            
            # Apply stylesheet with calculated pixel width
            self.ui.tabWidget.setStyleSheet(f"""
                QTabBar::tab {{
                    width: {tab_widget_width}px;
                    min-width: {width_per_tab}px;
                }}
            """)

    def emergency_stop(self):
        msg = Bool()
        msg.data = True
        self.ros_node.ui_emergency_stop_pub.publish(msg)
        self.joystick_terminate_change_page(False)
        if printlogger: self.ros_node.get_logger().info('Emergency Stop pressed')

    def home_position(self):
        msg = Bool()
        msg.data = True
        self.ros_node.ui_home_position_pub.publish(msg)
        if printlogger: self.ros_node.get_logger().info('Home Position pressed')
    
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
    
    def joystick_terminate_change_page(self, state = bool):
        if state:
            self.ui.stackedWidget.setCurrentIndex(0)
            if printlogger: self.ros_node.get_logger().info(f'Swithing to terminate page')
        elif not state:
            self.ui.stackedWidget.setCurrentIndex(1)
            if printlogger: self.ros_node.get_logger().info(f'Swithing to joystick page')

    def toggle_info_panel(self, checked):
        """Toggle info panel visibility when radio button is clicked"""
        if checked:
            self.ui.stackedWidget_Info.show()
            if printlogger: self.ros_node.get_logger().info('Info panel shown')
        else:
            self.ui.stackedWidget_Info.hide()
            if printlogger: self.ros_node.get_logger().info('Info panel hidden')

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



    def toggle_vision_state(self):
        self.camerafeed[1] = 1 - self.camerafeed[1]  # Toggle between 0 and 1
        if printlogger: self.ros_node.get_logger().info(f'Toggling Vision State {self.camerafeed[1]}')

    def switch_camera_type(self):
        self.camerafeed[0] = 1 - self.camerafeed[0]  # Toggle between 0 and 1
        if printlogger: self.ros_node.get_logger().info(f'Switching Camera Type {self.camerafeed[0]}')

    def reset_vision(self):
        self.ros_node.get_logger().info('Resetting vision areas')
        h,w = self.corrosion_area_add.shape
        self.corrosion_area_add = np.zeros((h, w), dtype=np.uint8)
        self.corrosion_area_remove = np.zeros((h, w), dtype=np.uint8)
        msg = Image()
        msg = self.numpy_to_image_msg(self.corrosion_area_remove, 'mono8')
        self.ros_node.ui_corrosion_area_remove_pub.publish(msg)
        msg = Image()
        msg = self.numpy_to_image_msg(self.corrosion_area_add, 'mono8')
        self.ros_node.ui_corrosion_area_add_pub.publish(msg)

        # Reset vision logic
        if printlogger: self.ros_node.get_logger().info('Resetting Vision')

    def undo_action(self):
        if len(self.undo_add_stack) > 0:
            # Restore previous state
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
            self.camerafeed[0] = 0
            self.camerafeed[1] = 0  # Show color feed on Movement tab
            self.tabindex = index
            self.ui.stackedWidget_Info.setCurrentIndex(0)  # Show Movement info
        elif index == 1:
            self.camerafeed[0] = 0
            self.camerafeed[1] = 1  # Show thresholded feed on Vision tab
            self.tabindex = index
            self.ui.stackedWidget_Info.setCurrentIndex(1)  # Show Vision info
            # Display the last received threshold frame if available
            if self.ros_node.last_Threshold_frame is not None:
                self.signal_emitter.data_signal.emit(f"Thresholded: {self.ros_node.last_Threshold_frame.shape[1]}x{self.ros_node.last_Threshold_frame.shape[0]}")
                self.signal_emitter.image_signal.emit(self.ros_node.last_Threshold_frame)
                if printlogger:
                    self.ros_node.get_logger().info('Displayed cached threshold frame on tab switch')
            else:
                if printlogger:
                    self.ros_node.get_logger().info('No cached threshold frame available yet')
        elif index == 2:
            self.camerafeed[0] = 0
            self.camerafeed[1] = 0
            self.ui.stackedWidget_Info.setCurrentIndex(2)  # Show System info
            pass  # System Information tab

    def on_joystick_touched(self):
        """Handle joystick touch event"""
        if printlogger: self.ros_node.get_logger().info('Joystick touched!')
        
        # Check if anything has been drawn in corrosion_area_add or corrosion_area_remove
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

    def on_joystick_released(self):
        """Handle joystick release event"""
        if printlogger: self.ros_node.get_logger().info('Joystick released!')
        # Add your custom logic here
        pass

    def on_joystick_moved(self, direction_tuple):
        """Handle joystick move event"""
        if direction_tuple != 0:
            direction, distance = direction_tuple
            self.ros_node.get_logger().info(f'Joystick moved: {direction.name}, distance: {distance:.2f}')
        # Add your custom logic here (e.g., send motor commands, update display, etc.)
        pass

    def on_data(self, data):
        self.ui.videoLabel.setText(data)
    
    def update_video_frame(self, img):
        """Convert numpy BGR array to QPixmap and display in videoLabel"""
        if img is None:
            return
        self.last_frame = img.copy()

        # Handle both color (3D) and grayscale (2D) images
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


