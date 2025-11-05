import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool  
import message_filters
import numpy as np
import cv2 as cv

kernel = np.ones((5, 5), np.uint8)
c = (480 / 2, 640 / 2)

printlogger = False
showImages = True

class CorrosionDetector(Node):
    def __init__(self):
        super().__init__('corrosion_detector')

        self.corrosion_scatter_plot = self.create_publisher(Float32MultiArray, 'corrosion_scatter_plot_pub', 10)
        self.corrosion_thresholding = self.create_publisher(Image, 'corrosion_thresholding_pub', 10)

        # Subscribe to both topics
        color_sub = message_filters.Subscriber(self, Image, 'realsense_camera_color_pub')
        depth_sub = message_filters.Subscriber(self, Image, 'realsense_camera_depth_pub')
         
        # Synchronize them based on timestamps
        sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        sync.registerCallback(self.image_match)

        self.ui_corrosion_area_accept_sub = self.create_subscription(Bool, 'ui_corrosion_area_accept_pub', self.ui_corrosion_area_accept_callback, 10)        
        self.ui_corrosion_add_sub = self.create_subscription(Image, 'ui_corrosion_area_add_pub', self.ui_corrosion_add_callback, 10)
        self.ui_corrosion_remove_sub = self.create_subscription(Image, 'ui_corrosion_area_remove_pub', self.ui_corrosion_remove_callback, 10)
        self.ROBODK_completion_notification = self.create_subscription(Bool, 'ROBODK_completion_notification_pub', self.ROBODK_completion_notification_callback, 10)
        self.corrosion_accepted = False  # in __init__
        self.running_status = False #To help with make a pipeline

        # Needs to be changed
        if printlogger: self.get_logger().info('Waiting for synchronized images...')

    def ui_corrosion_area_accept_callback(self, msg):
        # Receive UI command
        self.corrosion_accepted = msg.data
        if printlogger: self.get_logger().info(f'UI command received: {self.corrosion_accepted}')

    def ROBODK_completion_notification_callback(self, msg):
        if msg.data == True:
            self.running_status = False
            self.corrosion_accepted = False
            self.get_logger().info('ROBODK has completed the path, ready for new corrosion area')
            self.get_logger().info(f'State: corrosion_accepted={self.corrosion_accepted}, running_status={self.running_status}')


    def ui_corrosion_add_callback(self, msg):
        self.ui_corrosion_add = msg.data
        if printlogger: self.get_logger().info('UI command received: Add corrosion area')

    def ui_corrosion_remove_callback(self, msg):
        self.ui_corrosion_remove = msg.data
        if printlogger: self.get_logger().info('UI command received: Remove corrosion area')

    def numpy_to_image_msg(self, img, encoding):
        msg = Image()
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = img.shape[1] * img.itemsize * (3 if len(img.shape) == 3 else 1)
        msg.data = img.tobytes()
        return msg

    def image_match(self, color_msg, depth_msg):
        if printlogger: self.get_logger().info(f'Image and depth matched {color_msg.header.stamp.sec}.{color_msg.header.stamp.nanosec}')
        color_image = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, 3)
        depth_image = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width)

        if not self.corrosion_accepted or self.corrosion_accepted is None:
            depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)

            # Publish thresholded image
            thresholded_image = self.threshold_corrosion(color_image)
            color_threshold_image = color_image.copy()
            edge = cv.Canny(thresholded_image, 100, 200)
            color_threshold_image[edge > 0] = [0, 255, 0]
            self.corrosion_thresholding.publish(self.numpy_to_image_msg(color_threshold_image, "bgr8"))

            # Show images(Remove in production)
            if showImages:
                cv.imshow('Color', color_image)
                cv.imshow('Depth', depth_colormap)
                cv.imshow('Thresholded', thresholded_image)
                cv.imshow('Corrosion Area', color_threshold_image)
                cv.waitKey(1)
        elif self.corrosion_accepted and self.running_status == False:
            self.running_status = True
            self.get_logger().info('Corrosion area accepted')
            xyz_data = self.combine_and_transform(self.edge_to_scatter_plot(color_image), depth_image)
            msg = Float32MultiArray()
            msg.data = xyz_data.flatten().tolist()
            self.corrosion_scatter_plot.publish(msg)
        else:
            self.get_logger().info(f'Corrosion detection is already running, wait for ROBODK to complete {self.corrosion_accepted} {self.running_status}')
            #self.get_logger().info('A mistake has happened with UI command')
    
    def threshold_corrosion(self, image):
        # Thresholding in HSV color space to detect corrosion
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        h, s, v = cv.split(hsv)

        #HSV in OpenCV: H: 0-179, S: 0-255, V: 0-255
        h_min = int(0 * 179 / 255)    
        h_max = int(25 * 179 / 255) 
        
        thresh_h = cv.inRange(h, h_min, h_max) 
        _, thresh_s = cv.threshold(s, 75, 255, cv.THRESH_BINARY)
        _, thresh_v = cv.threshold(v, 25, 255, cv.THRESH_BINARY) 
        
        combined_mask = cv.bitwise_and(thresh_h, cv.bitwise_and(thresh_s, thresh_v))
        thresh_hsv = cv.merge([combined_mask, combined_mask, combined_mask])
        cv.imshow("Thresholded HSV", thresh_hsv)
        cv.waitKey(1)
        return thresh_hsv

    def clean_image(self, image):
        # Remove noise from thresholded image
        img_erode = cv.erode(image, kernel, iterations=1)
        img_dilate = cv.dilate(img_erode, kernel, iterations=3)
        img_final_erode = cv.erode(img_dilate, kernel, iterations=3)
        return img_final_erode

    def edge_to_scatter_plot(self, image, threshold1=100, threshold2=200):
        # Detect edges and convert to scatter plot data
        img_final_erode = cv.cvtColor(self.clean_image(self.threshold_corrosion(image)), cv.COLOR_BGR2GRAY)

        edges = cv.Canny(img_final_erode, threshold1, threshold2)
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        filled_mask = np.zeros_like(img_final_erode)
        cv.drawContours(filled_mask, contours, -1, 255, thickness=cv.FILLED)

        y_indices, x_indices = np.where(filled_mask > 0)
        scatter_data = np.column_stack((x_indices, y_indices))
        return scatter_data

    def combine_and_transform(self, scatter_data, depth, depthFiles=None):
        # Combine scatter plot data with depth to get XYZ coordinates
        depth_values = depth[scatter_data[:, 1], scatter_data[:, 0]]
        xyz_data = np.column_stack(((scatter_data[:, 0] - c[0]) * depth_values / (1.93/0.003), 
                                    (scatter_data[:, 1] - c[1]) * depth_values / (1.93/0.003), 
                                    depth_values))
        return xyz_data

    def destroy_node(self):
        # Cleanup 
        cv.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = CorrosionDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()