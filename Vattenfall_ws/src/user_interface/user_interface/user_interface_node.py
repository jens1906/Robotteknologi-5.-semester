import rclpy
import sys
import threading
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import pyqtSignal, QObject, Qt
from PyQt6.QtGui import QImage, QPixmap
from user_interface.GUI import Ui_MainWindow
import numpy as np
import cv2 as cv
import message_filters

Test = True
printlogger = True
showImages = True

class RosSignalEmitter(QObject):
    data_signal = pyqtSignal(str)           # existing: emits text
    image_signal = pyqtSignal(object)       # NEW: emits python objects (numpy arrays)

class UserInterfaceNode(Node):
    def __init__(self, signal_emitter, ui_instance=None):
        super().__init__('user_interface')
        self.signal_emitter = signal_emitter
        self.ui_instance = ui_instance  # Reference to UserInterface for state access
        self.ui_corrosion_area_accept_pub = self.create_publisher(Bool, 'ui_corrosion_area_accept_pub', 10)
        self.ui_corrosion_area_add_pub = self.create_publisher(Image, 'ui_corrosion_area_add_pub', 10)
        self.ui_corrosion_area_remove_pub = self.create_publisher(Image, 'ui_corrosion_area_remove_pub', 10)
        self.ui_home_position_pub = self.create_publisher(Bool, 'ui_home_position_pub', 10)
        self.ui_emergency_stop_pub = self.create_publisher(Bool, 'ui_emergency_stop_pub', 10)
        self.ui_terminate_pub = self.create_publisher(Bool, 'ui_terminate_pub', 10)

        self.corrosion_thresholding_pub = self.create_subscription(Image, 'corrosion_thresholding_pub', self.corrosion_thresholding_callback, 10)
        color_sub = message_filters.Subscriber(self, Image, 'realsense_camera_color_pub')
        depth_sub = message_filters.Subscriber(self, Image, 'realsense_camera_depth_pub')
         
        sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        sync.registerCallback(self.image_match)

        # Initialize UI components here (e.g., publishers/subscribers for UI commands)
        self.get_logger().info('User Interface Node Initialized')

    def image_match(self, color_msg, depth_msg):
        #if printlogger: self.get_logger().info(f'Image and depth matched {color_msg.header.stamp.sec}.{color_msg.header.stamp.nanosec}')
        color_image = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, 3)
        depth_image = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width)
        if self.ui_instance and self.ui_instance.detection_state == 0 and self.ui_instance.camera_type == 0:
            self.signal_emitter.data_signal.emit(f"Color: {depth_image.shape[1]}x{depth_image.shape[0]}")
            self.signal_emitter.image_signal.emit(color_image)
        elif self.ui_instance and self.ui_instance.camera_type == 1:
            self.signal_emitter.data_signal.emit(f"Depth: {depth_image.shape[1]}x{depth_image.shape[0]}")
            self.signal_emitter.image_signal.emit(depth_image)
        # Process images as needed (don't display with cv.imshow in Qt app)

    def corrosion_thresholding_callback(self, msg):
        corrosion_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if self.ui_instance and self.ui_instance.detection_state == 1 and self.ui_instance.camera_type == 0:
            self.signal_emitter.data_signal.emit(f"Thresholded: {msg.width}x{msg.height}")
            self.signal_emitter.image_signal.emit(corrosion_image)


    

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
        
        # State variables
        self.detection_state = 0  # 0 = Color, 1 = Thresholded
        self.camera_type = 0
        
        self.ui.videoLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.ui.Emergency_Stop.clicked.connect(self.emergency_stop)
        self.ui.Home_Position.clicked.connect(self.home_position)
        self.ui.RUN_1.clicked.connect(self.run_robot)
        self.ui.RUN_2.clicked.connect(self.run_robot)
        self.ui.Terminate.clicked.connect(self.terminate)
        self.ui.Vision_State.clicked.connect(self.toggle_vision_state)
        self.ui.Switch_Camera_Type.clicked.connect(self.switch_camera_type)
        self.ui.Reset.clicked.connect(self.reset_vision)
        self.ui.Undo.clicked.connect(self.undo_action)
        self.ui.Eraser.clicked.connect(self.erase_area)
        self.ui.Small_Pen.clicked.connect(lambda: self.set_custom_pen(1))
        self.ui.Medium_Pen.clicked.connect(lambda: self.set_custom_pen(2))
        self.ui.Large_Pen.clicked.connect(lambda: self.set_custom_pen(3))
        self.ui.tabWidget.currentChanged.connect(lambda index: self.tab_difference(index))

        # ROS setup
        self.signal_emitter = RosSignalEmitter()
        self.signal_emitter.data_signal.connect(self.on_data)
        self.signal_emitter.image_signal.connect(self.update_video_frame)
        self.ros_node = UserInterfaceNode(self.signal_emitter, self)  # Pass self for state access
    
    def emergency_stop(self):
        msg = Bool()
        msg.data = True
        self.ros_node.ui_emergency_stop_pub.publish(msg)
        if printlogger: self.ros_node.get_logger().info('Emergency Stop pressed')

    def home_position(self):
        msg = Bool()
        msg.data = True
        self.ros_node.ui_home_position_pub.publish(msg)
        if printlogger: self.ros_node.get_logger().info('Home Position pressed')
    
    def run_robot(self):
        msg = Bool()
        msg.data = True
        self.ros_node.ui_corrosion_area_accept_pub.publish(msg)
        if printlogger: self.ros_node.get_logger().info('Run Robot pressed')
    
    def terminate(self):
        msg = Bool()
        msg.data = True
        self.ros_node.ui_terminate_pub.publish(msg)
        if printlogger: self.ros_node.get_logger().info('Terminate pressed')

    def toggle_vision_state(self):
        self.detection_state = 1 - self.detection_state  # Toggle between 0 and 1
        if printlogger: self.ros_node.get_logger().info(f'Toggling Vision State {self.detection_state}')

    def switch_camera_type(self):
        self.camera_type = 1 - self.camera_type  # Toggle between 0 and 1
        if self.camera_type == 0:
            self.ui.Reset.setStyleSheet("background-color: #ffffff;")
            self.ui.Eraser.setStyleSheet("background-color: #ffffff;")
            self.ui.Undo.setStyleSheet("background-color: #ffffff;")

            if printlogger: self.ros_node.get_logger().info('Switching to Color Camera')
        else:
            self.ui.Reset.setStyleSheet("background-color: #636363;")
            self.ui.Eraser.setStyleSheet("background-color: #636363;")
            self.ui.Undo.setStyleSheet("background-color: #636363;")
            if printlogger: self.ros_node.get_logger().info('Switching to Depth Camera')

        if printlogger: self.ros_node.get_logger().info(f'Switching Camera Type {self.camera_type}')

    def reset_vision(self):
        # Reset vision logic
        if printlogger: self.ros_node.get_logger().info('Resetting Vision')

    def undo_action(self):
        # Undo action logic
        if printlogger: self.ros_node.get_logger().info('Undoing last action')

    def erase_area(self):
        if printlogger: self.ros_node.get_logger().info('Erase area requested')

    def set_custom_pen(self, size):
        if printlogger: self.ros_node.get_logger().info(f'Set pen size to {size}')

    def tab_difference(self, index):
        if printlogger: self.ros_node.get_logger().info(f'Tab changed to {index}')
        if index == 0:
            self.camera_type = 0
        elif index == 1:
            self.detection_state = 1







    def on_data(self, data):
        self.ui.videoLabel.setText(data)
    
    def update_video_frame(self, img):
        """Convert numpy BGR array to QPixmap and display in videoLabel"""
        if img is None:
            return
        
        # Handle both color (3D) and grayscale (2D) images
        if len(img.shape) == 3:
            # Color image (BGR)
            h, w, ch = img.shape
            bytes_per_line = ch * w
            qimage = QImage(img.data.tobytes(), w, h, bytes_per_line, QImage.Format.Format_BGR888).copy()
        else:
            # Grayscale image (2D)
            h, w = img.shape
            bytes_per_line = w
            qimage = QImage(img.data.tobytes(), w, h, bytes_per_line, QImage.Format.Format_Grayscale8).copy()
        
        pixmap = QPixmap.fromImage(qimage)
        
        scaled = pixmap.scaled(self.ui.videoLabel.size(), Qt.AspectRatioMode.KeepAspectRatio,
                               Qt.TransformationMode.SmoothTransformation)
        
        self.ui.videoLabel.setPixmap(scaled)


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = UserInterface()
    window.show()
    
    # Start ROS thread AFTER window is shown and Qt event loop is ready
    ros_thread = threading.Thread(target=lambda: rclpy.spin(window.ros_node), daemon=True)
    ros_thread.start()
    
    sys.exit(app.exec())
    window.ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
