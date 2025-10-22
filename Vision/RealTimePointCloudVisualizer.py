"""
Real-time XYZ point cloud visualization at 30 FPS using Open3D.
Optimized for large datasets with downsampling and efficient updates.
"""
import cv2 as cv
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import time
from collections import deque


class RealTimePointCloudVisualizer:
    def __init__(self, voxel_size=0.5, max_points=50000, show_every_nth_frame=1):
        """
        Args:
            voxel_size: Voxel size for downsampling (mm). Larger = fewer points.
            max_points: Maximum points to render (additional random downsampling if needed).
            show_every_nth_frame: Render every nth frame (1=all, 2=every other, etc.)
        """
        self.voxel_size = voxel_size
        self.max_points = max_points
        self.show_every_nth_frame = show_every_nth_frame
        self.frame_count = 0
        
        # Create Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("Real-Time Point Cloud", width=1280, height=720)
        
        # Create point cloud object (reuse for efficiency)
        self.pcd = o3d.geometry.PointCloud()
        self.geometry_added = False
        
        # FPS tracking
        self.fps_queue = deque(maxlen=30)
        self.last_time = time.time()
        
        # Setup RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        
        # Camera intrinsics (will update from first frame)
        self.fx = self.fy = 600  # approximate, will be updated
        self.cx = 320
        self.cy = 240
        
    def pixel_to_xyz(self, scatter_data, depth_image):
        """Convert 2D pixel coordinates to 3D XYZ (mm)."""
        depth_values = depth_image[scatter_data[:, 1], scatter_data[:, 0]]
        
        # Filter out zero/invalid depths
        valid_mask = depth_values > 0
        scatter_data = scatter_data[valid_mask]
        depth_values = depth_values[valid_mask]
        
        xyz = np.column_stack([
            (scatter_data[:, 0] - self.cx) * depth_values / self.fx,
            (scatter_data[:, 1] - self.cy) * depth_values / self.fy,
            depth_values
        ])
        return xyz
    
    def threshold_and_segment(self, color_image):
        """Simple HSV thresholding - replace with your edge_to_scatter_plot if needed."""
        hsv = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)
        
        # Example: detect objects (adjust thresholds for your use case)
        lower = np.array([0, 50, 50])
        upper = np.array([180, 255, 255])
        mask = cv.inRange(hsv, lower, upper)
        
        # Get all pixels in mask
        y_indices, x_indices = np.where(mask > 0)
        scatter_data = np.column_stack((x_indices, y_indices))
        
        return scatter_data
    
    def downsample_points(self, xyz, colors=None):
        """Downsample using voxel grid and random sampling."""
        if len(xyz) == 0:
            return xyz, colors
        
        # Create temporary point cloud for voxel downsampling
        temp_pcd = o3d.geometry.PointCloud()
        temp_pcd.points = o3d.utility.Vector3dVector(xyz)
        if colors is not None:
            temp_pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Voxel downsampling
        if self.voxel_size > 0:
            temp_pcd = temp_pcd.voxel_down_sample(voxel_size=self.voxel_size)
        
        xyz = np.asarray(temp_pcd.points)
        colors = np.asarray(temp_pcd.colors) if colors is not None and temp_pcd.has_colors() else None
        
        # Additional random downsampling if still too many points
        if len(xyz) > self.max_points:
            indices = np.random.choice(len(xyz), self.max_points, replace=False)
            xyz = xyz[indices]
            if colors is not None:
                colors = colors[indices]
        
        return xyz, colors
    
    def update_visualization(self, xyz, colors=None):
        """Update Open3D visualizer with new point cloud data."""
        # Downsample
        xyz, colors = self.downsample_points(xyz, colors)
        
        if len(xyz) == 0:
            return
        
        # Update point cloud geometry
        self.pcd.points = o3d.utility.Vector3dVector(xyz)
        if colors is not None:
            self.pcd.colors = o3d.utility.Vector3dVector(colors)
        else:
            # Default color: depth-based gradient
            z_norm = (xyz[:, 2] - xyz[:, 2].min()) / (xyz[:, 2].ptp() + 1e-6)
            colors_gradient = np.column_stack([z_norm, 1 - z_norm, np.zeros_like(z_norm)])
            self.pcd.colors = o3d.utility.Vector3dVector(colors_gradient)
        
        # Add geometry on first frame
        if not self.geometry_added:
            self.vis.add_geometry(self.pcd)
            self.geometry_added = True
            
            # Setup nice view
            view_ctrl = self.vis.get_view_control()
            view_ctrl.set_zoom(0.5)
        else:
            self.vis.update_geometry(self.pcd)
        
        # Non-blocking update
        self.vis.poll_events()
        self.vis.update_renderer()
    
    def calculate_fps(self):
        """Calculate and return current FPS."""
        current_time = time.time()
        fps = 1.0 / (current_time - self.last_time + 1e-6)
        self.fps_queue.append(fps)
        self.last_time = current_time
        return np.mean(self.fps_queue)
    
    def run(self):
        """Main loop: capture frames, process, and visualize."""
        print("Starting real-time visualization. Press 'q' in CV window or close Open3D window to quit.")
        print(f"Settings: voxel_size={self.voxel_size}mm, max_points={self.max_points}, render every {self.show_every_nth_frame} frame(s)")
        
        try:
            while True:
                # Capture frames
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                
                # Get intrinsics from first frame
                if self.frame_count == 0:
                    intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
                    self.fx, self.fy = intrinsics.fx, intrinsics.fy
                    self.cx, self.cy = intrinsics.ppx, intrinsics.ppy
                    print(f"Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")
                
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Segment/threshold to get pixels of interest
                scatter_data = self.threshold_and_segment(color_image)
                
                # Convert to 3D XYZ
                if len(scatter_data) > 0:
                    xyz = self.pixel_to_xyz(scatter_data, depth_image)
                    
                    # Get colors from original image (optional)
                    colors = color_image[scatter_data[:, 1], scatter_data[:, 0]][:, ::-1] / 255.0  # BGR to RGB, normalize
                    
                    # Update visualization (skip frames if configured)
                    if self.frame_count % self.show_every_nth_frame == 0:
                        self.update_visualization(xyz, colors)
                
                # Show 2D preview
                cv.imshow('Color Preview', color_image)
                
                # FPS display
                fps = self.calculate_fps()
                if self.frame_count % 10 == 0:
                    print(f"FPS: {fps:.1f} | Points: {len(scatter_data) if len(scatter_data) > 0 else 0} -> "
                          f"{len(self.pcd.points) if self.geometry_added else 0} (downsampled)")
                
                self.frame_count += 1
                
                # Check for quit
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # Check if Open3D window closed
                if not self.vis.poll_events():
                    break
                    
        finally:
            self.pipeline.stop()
            cv.destroyAllWindows()
            self.vis.destroy_window()
            print(f"\nTotal frames processed: {self.frame_count}")


if __name__ == "__main__":
    # Configuration
    visualizer = RealTimePointCloudVisualizer(
        voxel_size=1.0,          # mm - larger = fewer points, faster rendering
        max_points=100000,        # Cap on points after voxel downsampling
        show_every_nth_frame=1    # 1=all frames, 2=every other, etc.
    )
    
    visualizer.run()
