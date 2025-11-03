from build.corrosion_detection.build.lib.corrosion_detection.corrosion_detection_node import CorrosionDetector
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
import cv2 as cv

Test = True
printlogger = False
showImages = True

class UserInterface(Node):
    def __init__(self):
        super().__init__('user_interface')
        self.ui_corrosion_area_accept_pub = self.create_publisher(Bool, 'ui_corrosion_area_accept_pub', 10)
        self.ui_corrosion_area_add_pub = self.create_publisher(Image, 'ui_corrosion_area_add_pub', 10)
        self.ui_corrosion_area_remove_pub = self.create_publisher(Image, 'ui_corrosion_area_remove_pub', 10)

        # Initialize UI components here (e.g., publishers/subscribers for UI commands)
        self.get_logger().info('User Interface Node Initialized')

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


def main():
    rclpy.init()
    node = UserInterface()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
