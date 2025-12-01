import rclpy
import cv2 as cv
import numpy as np
import message_filters
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool
import tf2_ros
from tf2_ros import TransformException
from scipy.ndimage import median_filter
import os
from datetime import datetime  

# Camera intrinsics from /camera/color/camera_info
# K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
fx = 615.389  # Focal length X (pixels)
fy = 615.737  # Focal length Y (pixels)
cx = 324.183  # Principal point X (pixels)
cy = 242.415  # Principal point Y (pixels)

kernel = np.ones((5, 5), np.uint8)

showImages = True
printlogger = False


class CorrosionDetector(Node):
    def __init__(self):
        super().__init__('corrosion_detector')
        
        # QoS profile for image topics (reliable to match RealSense camera settings)
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
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
        if printlogger:
            self.get_logger().info('Hand-Eye Calibration Matrix loaded')
            self.get_logger().info(f'T_camera_to_ee:\n{self.T_camera_to_ee}')
        
        # Transform from world to base_link (from URDF: world→bordplade→mount→base_link)
        T_world_to_base = np.array([
            [-1.000, -0.000,  0.000,  0.200],
            [-0.000,  0.000, -1.000,  0.218],
            [-0.000, -1.000, -0.000,  -0.250],
            [ 0.000,  0.000,  0.000,  1.000]
        ])
        
        # Compute inverse: base_link to world (for transforming point clouds)
        R = T_world_to_base[:3, :3]  # Rotation part
        t = T_world_to_base[:3, 3]   # Translation part (in meters)
        
        # Inverse: R^T and -R^T @ t
        R_inv = R.T
        t_inv = -R_inv @ t
        
        self.T_base_to_world = np.eye(4)
        self.T_base_to_world[:3, :3] = R_inv
        self.T_base_to_world[:3, 3] = t_inv * 1000.0  # Convert translation to mm for consistency
        
        self.get_logger().info('Transform base_link → world (translation in mm):')
        self.get_logger().info(f'  Translation: [{self.T_base_to_world[0,3]:.3f}, {self.T_base_to_world[1,3]:.3f}, {self.T_base_to_world[2,3]:.3f}] mm')
        
        self.toolsizes = [30, 25]  # Example tool sizes in mm

        self.corrosion_thresholding = self.create_publisher(Image, '/corrosion/thresholding_pub', image_qos)
        self.corrosion_corrosion = self.create_publisher(Float32MultiArray, '/corrosion/corrosion', 10)
        self.corrosion_workspace = self.create_publisher(Float32MultiArray, '/corrosion/workspace', 10)
        self.corrosion_tool_size = self.create_publisher(Float32MultiArray, '/corrosion/tool_size', 10)
        self.ui_corrosion_area_accept_sub = self.create_subscription(Bool, '/ui/corrosion_area_accept_pub', self.ui_corrosion_area_accept_callback, 10)        
        self.ui_corrosion_add_sub = self.create_subscription(Image, '/ui/corrosion_area_add_pub', self.ui_corrosion_add_callback, image_qos)
        self.ui_corrosion_remove_sub = self.create_subscription(Image, '/ui/corrosion_area_remove_pub', self.ui_corrosion_remove_callback, image_qos)
        self.ui_emergency_stop_sub = self.create_subscription(Bool, '/ui/emergency_stop_pub', self.ui_emergency_stop_callback, 10)
        self.ui_terminate_pub_sub = self.create_subscription(Bool, '/ui/terminate_pub', self.ui_terminate_callback, 10)
        self.ui_connected_pub_sub = self.create_subscription(Bool, '/ui/connected_pub', self.ui_connected_callback, 10)
        self.ROBODK_completion_notification = self.create_subscription(Bool, '/ROBODK/completion_notification_pub', self.ROBODK_completion_notification_callback, 10)        
        
        # Initialize tf2 buffer and listener for proper transform lookups
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info('Initialized tf2 listener - waiting for robot transforms...')
        
        color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw', qos_profile=image_qos)
        depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw', qos_profile=image_qos)
        sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        sync.registerCallback(self.image_match)
        
        self.get_logger().info('Waiting for camera topics: /camera/color/image_raw and /camera/aligned_depth_to_color/image_raw')

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
        self.tf_received = False  # Flag to track if transform is available
        self.target_frame = 'base_link'  # Robot base frame
        self.source_frame = 'tool0'  # End-effector frame
        self.ui_corrosion_add = np.zeros((480, 640), np.uint8)
        self.ui_corrosion_remove = np.zeros((480, 640), np.uint8)

        # Create save directory if it doesn't exist
        # Use home directory to ensure consistent location regardless of install/source space
        home_dir = os.path.expanduser('~')
        self.save_dir = os.path.join(home_dir, 'Documents/GitHub/Robotteknologi-5.-semester/Vattenfall_ws/src/corrosion_detection/Saved_data')
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f'Save directory: {self.save_dir}')

        # Timer to periodically warn if /tf hasn't been received
        self.tf_warning_timer = self.create_timer(5.0, self.check_tf_status)

        if printlogger: self.get_logger().info('Initialized Corrosion Detector Node')

    def check_tf_status(self):
        """Periodically check if /tf has been received."""
        if not self.tf_received:
            self.get_logger().warn('Still waiting for /tf - robot transforms not available yet!')

    def lookup_transform(self):
        """Look up the current transform from tool0 to base_link using tf2."""
        try:
            # Look up the transform from source (tool0) to target (base_link)
            transform_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),  # Get latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            if not self.tf_received:
                self.get_logger().info(f'✓ Transform available: {self.source_frame} → {self.target_frame}')
                self.tf_received = True
            
            # Convert to homogeneous transformation matrix
            t = transform_stamped.transform
            self.combined_transformation_of_ur = self.transform_to_homogeneous_matrix(t)
            
            # Convert translation from metres to millimetres for consistency with camera points
            self.combined_transformation_of_ur[0:3, 3] *= 1000.0
            
            # Print end-effector → base transformation matrix
            if printlogger:
                self.get_logger().info('='*60)
                self.get_logger().info('End-Effector → Base Transform Matrix (from /tf):')
                self.get_logger().info(f'\n{self.combined_transformation_of_ur}')
                trans = self.combined_transformation_of_ur[0:3, 3]
                self.get_logger().info(f'Translation (m): [{trans[0]:.4f}, {trans[1]:.4f}, {trans[2]:.4f}]')
                self.get_logger().info(f'Translation (mm): [{trans[0]:.1f}, {trans[1]:.1f}, {trans[2]:.1f}]')
                self.get_logger().info('='*60)
            
            return True
            
        except TransformException as ex:
            if self.tf_received:  # Only log if we previously had it
                self.get_logger().warn(f'Transform lookup failed: {ex}')
            return False

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
        x, y, z, w = q.x, q.y, q.z, q.w
        norm = np.sqrt(x**2 + y**2 + z**2 + w**2)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm        
        R = np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - w*z),     2*(x*z + w*y)],
            [    2*(x*y + w*z), 1 - 2*(x**2 + z**2),     2*(y*z - w*x)],
            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])
        return R

    def transform_to_homogeneous_matrix(self, transform):
        tx, ty, tz = transform.translation.x, transform.translation.y, transform.translation.z
        R = self.quaternion_to_rotation_matrix(transform.rotation)
        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3] = [tx, ty, tz]
        return T

    def tf_static_callback(self, msg):
        if not self.tf_static_received:
            self.get_logger().info('✓ Received /tf_static - robot transforms now available!')
            self.tf_static_received = True
        
        if printlogger:
            self.get_logger().info(f'Received TF Static with {len(msg.transforms)} transforms')        
        self.get_logger().info(f'Received TF Static with {len(msg.transforms)} transforms')
        # Initialize combined transformation as identity matrix
        combined_transformation = np.eye(4)
        
        # Process each transform in the message and multiply them
        for transform in msg.transforms:
            parent_frame = transform.header.frame_id
            child_frame = transform.child_frame_id
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            T = self.transform_to_homogeneous_matrix(transform.transform)
            combined_transformation = combined_transformation @ T
            
            if printlogger:
                self.get_logger().info(
                    f'Transform: {parent_frame} -> {child_frame}\n'
                    f'  Position: x={translation.x:.3f}, y={translation.y:.3f}, z={translation.z:.3f}\n'
                    f'  Rotation: x={rotation.x:.3f}, y={rotation.y:.3f}, z={rotation.z:.3f}, w={rotation.w:.3f}'
                )
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
        # Look up the current transform (updates every frame with robot movement)
        if not self.lookup_transform():
            if printlogger:
                self.get_logger().info('Skipping image processing - transform not available')
            return        
        if printlogger: self.get_logger().info(f'Image and depth matched {color_msg.header.stamp.sec}.{color_msg.header.stamp.nanosec}')
        color_image = cv.cvtColor(np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, 3), cv.COLOR_RGB2BGR)
        depth_image = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width)

        # Check if UI masks changed
        ui_changed = self.arrays_differ(self.last_added_area, self.ui_corrosion_add) or \
                    self.arrays_differ(self.last_removed_area, self.ui_corrosion_remove)

        if not self.corrosion_accepted:
            # Process on: first frame, UI change, or movement change, OR subscriber increased
            should_process = (not self.first_frame_received) or ui_changed or self.movement_change or self.ui_connected_state==False
            
            if should_process:
                color_threshold_image = color_image.copy()
                edge = self.Threshold_to_edge_with_edits(color_image)
                edge = cv.dilate(edge, cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5)), iterations=1) 
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
            
            # Visualize the 3D point cloud before publishing
            if showImages:
                self.visualize_point_cloud(xyz_data, xyz_offset, color_image)
            
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

            # Save to file original image, depth, corrosion point cloud and workspace pointcloud for record keeping
            self.save_data(color_image, depth_image, xyz_data, xyz_offset)





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

    def Threshold_to_edge_with_edits(self, image):
        thresholded_image = self.threshold_corrosion(image)
        thresh_gray = cv.cvtColor(thresholded_image, cv.COLOR_BGR2GRAY)
        combined_mask = cv.bitwise_or(thresh_gray, self.ui_corrosion_add)
        combined_mask = cv.bitwise_and(combined_mask, cv.bitwise_not(self.ui_corrosion_remove))
        cleaned_mask = self.clean_image(cv.merge([combined_mask, combined_mask, combined_mask]))
        edge = cv.Canny(cleaned_mask, 100, 200)
        return edge

    def edge_to_scatter_plot(self, image, threshold1=100, threshold2=200):
        edges = self.Threshold_to_edge_with_edits(image)
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        filled_mask = np.zeros_like(image[:, :, 0])
        cv.drawContours(filled_mask, contours, -1, 255, thickness=cv.FILLED)
        
        # Apply offset by dilating the filled mask to create workspace boundary
        # Workspace scale factor: 3.0 = 3x bigger workspace area around corrosion
        workspace_scale = 1.0
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
            
            return 
            
    def median_filter_depth(self, depth, kernel_size=9):
        # Apply median filter to depth image to reduce noise
        # scipy.ndimage.median_filter works with any data type (including uint16)
        # kernel_size is used as the filter size for the median filter
        filtered_depth = median_filter(depth, size=kernel_size)
        return filtered_depth
    

    def combine_and_transform(self, scatter_data_tuple, depth, depthFiles=None):
        # Unpack the tuple (original, offset)
        scatter_data_original, scatter_data_offset = scatter_data_tuple
        
        depth = self.median_filter_depth(depth, kernel_size=9)

        # Check if scatter_data is empty
        if scatter_data_offset is None or len(scatter_data_offset) == 0:
            if printlogger: self.get_logger().warn('No scatter data to transform')
            return np.array([]), np.array([])
        
        # Transform ORIGINAL data (camera pixel + depth -> camera XYZ)
        depth_values_orig = depth[scatter_data_original[:, 1], scatter_data_original[:, 0]]
        xyz_camera_original = np.column_stack(((scatter_data_original[:, 0] - cx) * depth_values_orig / fx, 
                                                (scatter_data_original[:, 1] - cy) * depth_values_orig / fy, 
                                                depth_values_orig))
        
        # Transform OFFSET data (camera pixel + depth -> camera XYZ)
        depth_values_offset = depth[scatter_data_offset[:, 1], scatter_data_offset[:, 0]]
        xyz_camera_offset = np.column_stack(((scatter_data_offset[:, 0] - cx) * depth_values_offset / fx, 
                                             (scatter_data_offset[:, 1] - cy) * depth_values_offset / fy, 
                                             depth_values_offset))
        
        # Apply hand-eye calibration to transform from camera frame to base_link frame
        xyz_base_original = self.apply_hand_eye_transform(xyz_camera_original)
        xyz_base_offset = self.apply_hand_eye_transform(xyz_camera_offset)
        
        if printlogger:
            self.get_logger().info(f'Transformed {len(xyz_base_original)} original points and {len(xyz_base_offset)} offset points to base_link frame')
        
        # Transform from base_link to world frame (both points are in mm)
        if len(xyz_base_original) > 0:
            ones_orig = np.ones((xyz_base_original.shape[0], 1))
            xyz_base_homogeneous_orig = np.hstack([xyz_base_original, ones_orig])
            xyz_world_homogeneous_orig = (self.T_base_to_world @ xyz_base_homogeneous_orig.T).T
            xyz_world_original = xyz_world_homogeneous_orig[:, :3]
        else:
            xyz_world_original = xyz_base_original
        
        if len(xyz_base_offset) > 0:
            ones_offset = np.ones((xyz_base_offset.shape[0], 1))
            xyz_base_homogeneous_offset = np.hstack([xyz_base_offset, ones_offset])
            xyz_world_homogeneous_offset = (self.T_base_to_world @ xyz_base_homogeneous_offset.T).T
            xyz_world_offset = xyz_world_homogeneous_offset[:, :3]
        else:
            xyz_world_offset = xyz_base_offset
        
        if printlogger:
            self.get_logger().info(f'Transformed to world frame: {len(xyz_world_original)} corrosion points, {len(xyz_world_offset)} workspace points')
            if len(xyz_world_original) > 0:
                self.get_logger().info(f'  Corrosion X range: [{xyz_world_original[:,0].min():.1f}, {xyz_world_original[:,0].max():.1f}] mm')
                self.get_logger().info(f'  Corrosion Y range: [{xyz_world_original[:,1].min():.1f}, {xyz_world_original[:,1].max():.1f}] mm')
                self.get_logger().info(f'  Corrosion Z range: [{xyz_world_original[:,2].min():.1f}, {xyz_world_original[:,2].max():.1f}] mm')
        
        return xyz_world_original, xyz_world_offset
    
    def visualize_point_cloud(self, xyz_corrosion, xyz_workspace, color_image):
        """
        Visualize the 3D point cloud in world frame coordinates.
        Shows both the corrosion area and workspace boundary.
        """
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            
            fig = plt.figure(figsize=(15, 5))
            
            # 3D scatter plot
            ax1 = fig.add_subplot(131, projection='3d')
            if len(xyz_corrosion) > 0:
                ax1.scatter(xyz_corrosion[:, 0], xyz_corrosion[:, 1], xyz_corrosion[:, 2], 
                           c='red', marker='.', s=1, label='Corrosion Area')
            if len(xyz_workspace) > 0:
                ax1.scatter(xyz_workspace[:, 0], xyz_workspace[:, 1], xyz_workspace[:, 2], 
                           c='blue', marker='.', s=0.5, alpha=0.3, label='Workspace')
            ax1.set_xlabel('X (mm)')
            ax1.set_ylabel('Y (mm)')
            ax1.set_zlabel('Z (mm)')
            ax1.set_title('3D Point Cloud (World Frame)')
            ax1.legend()
            
            # Top view (XY plane)
            ax2 = fig.add_subplot(132)
            if len(xyz_corrosion) > 0:
                ax2.scatter(xyz_corrosion[:, 0], xyz_corrosion[:, 1], c='red', marker='.', s=1, label='Corrosion')
            if len(xyz_workspace) > 0:
                ax2.scatter(xyz_workspace[:, 0], xyz_workspace[:, 1], c='blue', marker='.', s=0.5, alpha=0.3, label='Workspace')
            ax2.set_xlabel('X (mm)')
            ax2.set_ylabel('Y (mm)')
            ax2.set_title('Top View (XY Plane)')
            ax2.axis('equal')
            ax2.legend()
            ax2.grid(True)
            
            # Original 2D detection
            ax3 = fig.add_subplot(133)
            ax3.imshow(cv.cvtColor(color_image, cv.COLOR_BGR2RGB))
            if hasattr(self, 'scatter_data_original') and len(self.scatter_data_original) > 0:
                ax3.scatter(self.scatter_data_original[:, 0], self.scatter_data_original[:, 1], 
                           c='red', marker='.', s=1, alpha=0.5)
            ax3.set_title('2D Detection (Camera View)')
            ax3.axis('off')
            
            plt.tight_layout()
            
            # Log statistics
            self.get_logger().info(f'Point cloud statistics:')
            self.get_logger().info(f'  Corrosion points: {len(xyz_corrosion)}')
            self.get_logger().info(f'  Workspace points: {len(xyz_workspace)}')
            if len(xyz_corrosion) > 0:
                self.get_logger().info(f'  X range: [{xyz_corrosion[:, 0].min():.1f}, {xyz_corrosion[:, 0].max():.1f}] mm')
                self.get_logger().info(f'  Y range: [{xyz_corrosion[:, 1].min():.1f}, {xyz_corrosion[:, 1].max():.1f}] mm')
                self.get_logger().info(f'  Z range: [{xyz_corrosion[:, 2].min():.1f}, {xyz_corrosion[:, 2].max():.1f}] mm')
            
            plt.show(block=False)
            plt.pause(200)
            plt.close(fig)
            
        except ImportError:
            self.get_logger().warn('matplotlib not available, skipping visualization')
        except Exception as e:
            self.get_logger().error(f'Visualization error: {e}')

    def save_data(self, color_image, depth_image, xyz_corrosion, xyz_workspace):
        """
        Save color image, depth image, corrosion point cloud, and workspace point cloud to files.
        Creates a timestamped folder for each capture to keep related data together.
        """
        try:
            # Generate timestamp for unique folder name
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Create a timestamped subfolder for this capture
            capture_dir = os.path.join(self.save_dir, f'capture_{timestamp}')
            os.makedirs(capture_dir, exist_ok=True)
            
            # Save color image
            color_filename = os.path.join(capture_dir, 'color.png')
            cv.imwrite(color_filename, color_image)
            
            # Save depth image as numpy array (preserves full precision)
            depth_filename = os.path.join(capture_dir, 'depth.npy')
            np.save(depth_filename, depth_image)
            
            # Save corrosion point cloud
            corrosion_filename = os.path.join(capture_dir, 'corrosion_pointcloud.npy')
            np.save(corrosion_filename, xyz_corrosion)
            
            # Save workspace point cloud
            workspace_filename = os.path.join(capture_dir, 'workspace_pointcloud.npy')
            np.save(workspace_filename, xyz_workspace)
            
            self.get_logger().info(f'Data saved successfully to: {capture_dir}')
            self.get_logger().info(f'  - Color image: color.png')
            self.get_logger().info(f'  - Depth data: depth.npy')
            self.get_logger().info(f'  - Corrosion points: corrosion_pointcloud.npy ({len(xyz_corrosion)} points)')
            self.get_logger().info(f'  - Workspace points: workspace_pointcloud.npy ({len(xyz_workspace)} points)')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save data: {e}')

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