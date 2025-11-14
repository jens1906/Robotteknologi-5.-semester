import rclpy
import cv2 as cv
import numpy as np
import message_filters
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool  

c = (480 / 2, 640 / 2)
kernel = np.ones((5, 5), np.uint8)

showImages = True
printlogger = False


class CorrosionDetector(Node):
    def __init__(self):
        super().__init__('corrosion_detector')
        
        # QoS profile for image topics (best effort for network transmission)
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.corrosion_thresholding = self.create_publisher(Image, '/corrosion/thresholding_pub', image_qos)
        self.corrosion_scatter_plot = self.create_publisher(Float32MultiArray, '/corrosion/scatter_plot_pub', 10)
        self.ui_corrosion_add = np.zeros((480, 640), np.uint8)
        self.ui_corrosion_remove = np.zeros((480, 640), np.uint8)
        # Subscribers
        self.ui_corrosion_area_accept_sub = self.create_subscription(Bool, '/ui/corrosion_area_accept_pub', self.ui_corrosion_area_accept_callback, 10)        
        self.ui_corrosion_add_sub = self.create_subscription(Image, '/ui/corrosion_area_add_pub', self.ui_corrosion_add_callback, image_qos)
        self.ui_corrosion_remove_sub = self.create_subscription(Image, '/ui/corrosion_area_remove_pub', self.ui_corrosion_remove_callback, image_qos)
        self.ui_emergency_stop_sub = self.create_subscription(Bool, '/ui/emergency_stop_pub', self.ui_emergency_stop_callback, 10)
        self.ui_terminate_pub_sub = self.create_subscription(Bool, '/ui/terminate_pub', self.ui_terminate_callback, 10)
        self.ui_connected_pub_sub = self.create_subscription(Bool, '/ui/connected_pub', self.ui_connected_callback, 10)
        self.ROBODK_completion_notification = self.create_subscription(Bool, '/ROBODK/completion_notification_pub', self.ROBODK_completion_notification_callback, 10)
        color_sub = message_filters.Subscriber(self, Image, '/realsense/camera_color_pub', qos_profile=image_qos)
        depth_sub = message_filters.Subscriber(self, Image, '/realsense/camera_depth_pub', qos_profile=image_qos)
        sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        sync.registerCallback(self.image_match)

        self.corrosion_accepted = False  
        self.running_status = False 
        self.last_frame = None
        self.last_added_area = None
        self.last_removed_area = None
        self.movement_change = True
        self.first_frame_received = False
        self.last_subscriber_count = 0  # Track subscriber count changes
        self.ui_connected_state = False

        if printlogger: self.get_logger().info('Initialized Corrosion Detector Node')

    def ui_corrosion_area_accept_callback(self, msg):
        self.corrosion_accepted = msg.data
        if printlogger: self.get_logger().info(f'UI command received: {self.corrosion_accepted}')

    def ui_connected_callback(self, msg):
        self.ui_connected_state = msg.data
        if printlogger: self.get_logger().info(f'UI connected state updated: {self.ui_connected_state}')


    def ROBODK_completion_notification_callback(self, msg):
        if msg.data == True:
            self.running_status = False
            self.corrosion_accepted = False
            if printlogger:
               self.get_logger().info('ROBODK has completed the path, ready for new corrosion area')
        if printlogger:
           self.get_logger().info(f'State: corrosion_accepted={self.corrosion_accepted}, running_status={self.running_status}')

    def ui_emergency_stop_callback(self, msg):
        self.running_status = False
        self.corrosion_accepted = False
        self.get_logger().info('Emergency stop received, stopping corrosion detection')
        if printlogger:
           self.get_logger().info(f'State: corrosion_accepted={self.corrosion_accepted}, running_status={self.running_status}')

    def ui_terminate_callback(self, msg):
        self.running_status = False
        self.corrosion_accepted = False
        self.get_logger().info('Terminate command received, stopping activities')

    def image_msg_to_numpy(self, msg):
        """Convert ROS Image message to numpy array."""
        if msg.encoding == 'mono8':
            # Single channel uint8
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        else:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")

    def ui_corrosion_add_callback(self, msg):
        self.ui_corrosion_add = self.image_msg_to_numpy(msg)

        if printlogger: self.get_logger().info('UI command received: Add corrosion area')

    def ui_corrosion_remove_callback(self, msg):
        self.ui_corrosion_remove = self.image_msg_to_numpy(msg)
        if printlogger: self.get_logger().info('UI command received: Remove corrosion area')

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

    @staticmethod
    def arrays_differ(a, b):
        """Check if two numpy arrays differ, handling None cases."""
        # Both None = no difference
        if a is None and b is None:
            return False
        # One is None = different
        if a is None or b is None:
            return True
        # Different shapes = different
        if a.shape != b.shape:
            return True
        # Use np.array_equal for exact element-wise equality
        return not np.array_equal(a, b)


    def image_match(self, color_msg, depth_msg):
        if printlogger: self.get_logger().info(f'Image and depth matched {color_msg.header.stamp.sec}.{color_msg.header.stamp.nanosec}')
        
        color_image = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, 3)
        depth_image = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width)


        # Check if UI masks changed
        ui_changed = self.arrays_differ(self.last_added_area, self.ui_corrosion_add) or \
                    self.arrays_differ(self.last_removed_area, self.ui_corrosion_remove)

        if not self.corrosion_accepted:
            # Process on: first frame, UI change, or movement change, OR subscriber increased
            should_process = (not self.first_frame_received) or ui_changed or self.movement_change or self.ui_connected_state==False
            
            if should_process:
                thresholded_image = self.threshold_corrosion(color_image)
                color_threshold_image = color_image.copy()

                # Convert threshold to grayscale (single channel)
                thresh_gray = cv.cvtColor(thresholded_image, cv.COLOR_BGR2GRAY)

                # Combine: add UI painted areas, remove UI erased areas
                combined_mask = cv.bitwise_or(thresh_gray, self.ui_corrosion_add)
                combined_mask = cv.bitwise_and(combined_mask, cv.bitwise_not(self.ui_corrosion_remove))

                # Clean the combined mask (your existing morphology)
                cleaned_mask = self.clean_image(cv.merge([combined_mask, combined_mask, combined_mask]))
                cleaned_gray = cv.cvtColor(cleaned_mask, cv.COLOR_BGR2GRAY)

                # Edge detection on the cleaned, combined mask
                edge = cv.Canny(cleaned_gray, 100, 200)
                
                # Dilate edges to make them thicker (5 pixels)
                edge_kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
                edge = cv.dilate(edge, edge_kernel, iterations=2)

                # Overlay green edges on original color image
                color_threshold_image[edge > 0] = [0, 255, 0]
                
                # Save state for next comparison
                self.last_frame = color_threshold_image.copy()
                self.last_added_area = self.ui_corrosion_add.copy()
                self.last_removed_area = self.ui_corrosion_remove.copy()
                self.movement_change = False
                self.first_frame_received = True
                
                # Publish processed frame
                self.corrosion_thresholding.publish(self.numpy_to_image_msg(color_threshold_image, "bgr8"))

        elif self.corrosion_accepted and self.running_status == False:
            self.running_status = True
            xyz_data = self.combine_and_transform(self.edge_to_scatter_plot(color_image), depth_image)
            msg = Float32MultiArray()
            msg.data = xyz_data.flatten().tolist()
            self.corrosion_scatter_plot.publish(msg)
            if printlogger: self.get_logger().info('Corrosion area accepted')
        else:
            if printlogger: self.get_logger().info(f'Corrosion detection is already running, wait for ROBODK to complete {self.corrosion_accepted} {self.running_status}')


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
        
        # Dilate edges to make them thicker (5 pixels)
        edge_kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        edges = cv.dilate(edges, edge_kernel, iterations=2)
        
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
        


        cv.destroyAllWindows()
        super().destroy_node()
        


def main():
    rclpy.init()
    node = CorrosionDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()