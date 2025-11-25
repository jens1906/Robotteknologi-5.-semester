import rclpy
import cv2 as cv
import numpy as np
import message_filters
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool
from tf2_msgs.msg import TFMessage  

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
        
        # Hand-Eye Calibration Matrix (Camera to End Effector)
        # This transforms points from camera frame to robot end-effector frame
        self.T_camera_to_ee = np.array([
            [-0.996, -0.088, -0.009, 39.945],
            [ 0.088, -0.996, -0.005, 47.026],
            [-0.008, -0.005,  1.000, -6.355],
            [ 0.000,  0.000,  0.000,  1.000]
        ])
        self.get_logger().info('Hand-Eye Calibration Matrix loaded')
        self.get_logger().info(f'T_camera_to_ee:\n{self.T_camera_to_ee}')
        
        self.toolsizes = [30, 25]  # Example tool sizes in mm

        self.corrosion_thresholding = self.create_publisher(Image, '/corrosion/thresholding_pub', image_qos)
        self.corrosion_corrosion = self.create_publisher(Float32MultiArray, '/corrosion/corrosion', 10)
        self.corrosion_workspace = self.create_publisher(Float32MultiArray, '/corrosion/workspace', 10)
        self.corrosion_tool_size = self.create_publisher(Float32MultiArray, '/corrosion/tool_size', 10)

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
        
        self.tf_static_sub = self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, 10)
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
        self.last_corrosion_threshold_image = None
        self.combined_transformation_of_ur = None

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

    def quaternion_to_rotation_matrix(self, q):
        """
        Convert a quaternion to a 3x3 rotation matrix.
        
        Args:
            q: quaternion with attributes x, y, z, w
            
        Returns:
            3x3 rotation matrix as numpy array
        """
        x, y, z, w = q.x, q.y, q.z, q.w
        
        # Normalize quaternion
        norm = np.sqrt(x**2 + y**2 + z**2 + w**2)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # Convert to rotation matrix
        R = np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - w*z),     2*(x*z + w*y)],
            [    2*(x*y + w*z), 1 - 2*(x**2 + z**2),     2*(y*z - w*x)],
            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])
        return R

    def transform_to_homogeneous_matrix(self, transform):
        """
        Convert a Transform (translation + rotation) to a 4x4 homogeneous transformation matrix.
        
        Args:
            transform: geometry_msgs/Transform with translation and rotation
            
        Returns:
            4x4 homogeneous transformation matrix
        """
        # Extract translation
        tx = transform.translation.x
        ty = transform.translation.y
        tz = transform.translation.z
        
        # Convert quaternion rotation to 3x3 rotation matrix
        R = self.quaternion_to_rotation_matrix(transform.rotation)
        
        # Create 4x4 homogeneous transformation matrix
        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3] = [tx, ty, tz]
        
        return T

    def tf_static_callback(self, msg):
        """
        Callback for /tf_static topic.
        This receives static transform information between frames and combines them.
        
        Args:
            msg: TFMessage containing TransformStamped objects
        """
        if printlogger:
            self.get_logger().info(f'Received TF Static with {len(msg.transforms)} transforms')
        
        # Initialize combined transformation as identity matrix
        combined_transformation = np.eye(4)
        
        # Process each transform in the message and multiply them
        for transform in msg.transforms:
            parent_frame = transform.header.frame_id
            child_frame = transform.child_frame_id
            
            # Extract translation and rotation
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # Convert transform to 4x4 homogeneous matrix
            T = self.transform_to_homogeneous_matrix(transform.transform)
            
            # Multiply with accumulated transformation
            combined_transformation = combined_transformation @ T
            
            if printlogger:
                self.get_logger().info(
                    f'Transform: {parent_frame} -> {child_frame}\n'
                    f'  Position: x={translation.x:.3f}, y={translation.y:.3f}, z={translation.z:.3f}\n'
                    f'  Rotation: x={rotation.x:.3f}, y={rotation.y:.3f}, z={rotation.z:.3f}, w={rotation.w:.3f}'
                )
        
        # Save the combined transformation to instance variable
        self.combined_transformation_of_ur = combined_transformation
        
        if printlogger:
            self.get_logger().info('Combined Transformation Matrix (End-Effector to Base):')
            self.get_logger().info(f'\n{self.combined_transformation_of_ur}')
            self.get_logger().info(f'Position: x={self.combined_transformation_of_ur[0, 3]:.3f}, y={self.combined_transformation_of_ur[1, 3]:.3f}, z={self.combined_transformation_of_ur[2, 3]:.3f}')

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
                
                # Dilate edges to create an offset/buffer (expand outward by ~5 pixels)
                edge_kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))  # Larger kernel for bigger offset
                edge = cv.dilate(edge, edge_kernel, iterations=1)  # 1 iteration with large kernel

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
            xyz_data, xyz_offset = self.combine_and_transform(self.edge_to_scatter_plot(color_image), depth_image)
            msg = Float32MultiArray()
            msg.data = xyz_data.flatten().tolist()
            self.corrosion_corrosion.publish(msg)

            msg = Float32MultiArray()
            msg.data = xyz_offset.flatten().tolist()
            self.corrosion_workspace.publish(msg)
            if printlogger: self.get_logger().info('Corrosion area accepted')

            msg = Float32MultiArray()
            msg.data = self.toolsizes
            self.corrosion_tool_size.publish(msg)
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
        thresholded_image = self.threshold_corrosion(image)

        # Convert threshold to grayscale (single channel)
        thresh_gray = cv.cvtColor(thresholded_image, cv.COLOR_BGR2GRAY)

        # Combine: add UI painted areas, remove UI erased areas
        combined_mask = cv.bitwise_or(thresh_gray, self.ui_corrosion_add)
        combined_mask = cv.bitwise_and(combined_mask, cv.bitwise_not(self.ui_corrosion_remove))

        # Clean the combined mask (your existing morphology)
        cleaned_mask = self.clean_image(cv.merge([combined_mask, combined_mask, combined_mask]))
        cleaned_gray = cv.cvtColor(cleaned_mask, cv.COLOR_BGR2GRAY)
        edges = cv.Canny(cleaned_gray, 100, 200)
        
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        filled_mask = np.zeros_like(image[:, :, 0])
        cv.drawContours(filled_mask, contours, -1, 255, thickness=cv.FILLED)
        
        # Apply offset by dilating the filled mask to create workspace boundary
        # Workspace scale factor: 3.0 = 3x bigger workspace area around corrosion
        workspace_scale = 3.0
        offset_kernel_size = int(max(self.toolsizes) / 0.8 * workspace_scale) * 2 + 1  # Scaled kernel size
        offset_kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (offset_kernel_size, offset_kernel_size))
        filled_mask_offset = cv.dilate(filled_mask, offset_kernel, iterations=1)

        # Return both original and offset masks for visualization
        y_indices_orig, x_indices_orig = np.where(filled_mask > 0)
        scatter_data_original = np.column_stack((x_indices_orig, y_indices_orig))
        
        y_indices, x_indices = np.where(filled_mask_offset > 0)
        scatter_data = np.column_stack((x_indices, y_indices))
        
        # Store original for visualization
        self.scatter_data_original = scatter_data_original
        
        return scatter_data_original, scatter_data

    def apply_hand_eye_transform(self, xyz_camera):
        """
        Apply hand-eye calibration and UR transformation to transform points from camera frame to robot base frame.
        
        Transformation chain:
        1. Camera frame -> End-effector frame (via T_camera_to_ee)
        2. End-effector frame -> Robot base frame (via combined_transformation_of_ur)
        
        Combined: T_total = combined_transformation_of_ur @ T_camera_to_ee
        
        Args:
            xyz_camera: (N, 3) array of points in camera coordinate frame
            
        Returns:
            xyz_base: (N, 3) array of points in robot base coordinate frame (or end-effector if UR transform not available)
        """
        if len(xyz_camera) == 0:
            return xyz_camera
        
        # Convert to homogeneous coordinates (N, 4)
        ones = np.ones((xyz_camera.shape[0], 1))
        xyz_homogeneous = np.hstack([xyz_camera, ones])
        
        # Combine transformations: Camera -> End-effector -> Base
        if self.combined_transformation_of_ur is not None:
            # Combined transformation matrix
            T_total = self.combined_transformation_of_ur @ self.T_camera_to_ee
            
            # Apply combined transformation in one step: T_total @ points^T -> (4, N)
            xyz_base_homogeneous = (T_total @ xyz_homogeneous.T).T
            
            # Convert back to Cartesian coordinates (N, 3)
            xyz_base = xyz_base_homogeneous[:, :3]
            self.get_logger().info('Applied combined UR transformation to points')
            return xyz_base
        else:
            # If UR transformation not available, only apply hand-eye calibration
            xyz_ee_homogeneous = (self.T_camera_to_ee @ xyz_homogeneous.T).T
            xyz_ee = xyz_ee_homogeneous[:, :3]
            self.get_logger().warn('UR transformation not available, returning end-effector frame coordinates only')
            if printlogger:
                self.get_logger().warn('Combined UR transformation not available, returning end-effector frame coordinates')
            
            return xyz_ee

    def combine_and_transform(self, scatter_data_tuple, depth, depthFiles=None):
        # Unpack the tuple (original, offset)
        scatter_data_original, scatter_data_offset = scatter_data_tuple
        
        # Check if scatter_data is empty
        if scatter_data_offset is None or len(scatter_data_offset) == 0:
            if printlogger: self.get_logger().warn('No scatter data to transform')
            return np.array([]), np.array([])
        
        # Transform ORIGINAL data (camera pixel + depth -> camera XYZ)
        depth_values_orig = depth[scatter_data_original[:, 1], scatter_data_original[:, 0]]
        xyz_camera_original = np.column_stack(((scatter_data_original[:, 0] - c[0]) * depth_values_orig / (1.93/0.003), 
                                                (scatter_data_original[:, 1] - c[1]) * depth_values_orig / (1.93/0.003), 
                                                depth_values_orig))
        
        # Transform OFFSET data (camera pixel + depth -> camera XYZ)
        depth_values_offset = depth[scatter_data_offset[:, 1], scatter_data_offset[:, 0]]
        xyz_camera_offset = np.column_stack(((scatter_data_offset[:, 0] - c[0]) * depth_values_offset / (1.93/0.003), 
                                             (scatter_data_offset[:, 1] - c[1]) * depth_values_offset / (1.93/0.003), 
                                             depth_values_offset))
        
        # Apply hand-eye calibration to transform from camera frame to end-effector frame
        xyz_ee_original = self.apply_hand_eye_transform(xyz_camera_original)
        xyz_ee_offset = self.apply_hand_eye_transform(xyz_camera_offset)
        
        if printlogger:
            self.get_logger().info(f'Transformed {len(xyz_ee_original)} original points and {len(xyz_ee_offset)} offset points to end-effector frame')
        
        return xyz_ee_original, xyz_ee_offset

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