"""
Path Visualization Node

Subscribes to /path/uv_path and visualizes the UV path using matplotlib.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class PathVisualizer(Node):
    """Visualize UV path from path planning node."""
    
    def __init__(self):
        super().__init__('path_visualizer')
        
        # Data holder
        self.uv_path = None
        self.updated = False
        
        # Subscribe to UV path
        self.create_subscription(
            Float64MultiArray,
            '/path/uv_path',
            self.path_callback,
            10
        )
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.line, = self.ax.plot([], [], 'b-', linewidth=2, label='UV Path')
        self.start_point, = self.ax.plot([], [], 'go', markersize=10, label='Start')
        self.end_point, = self.ax.plot([], [], 'ro', markersize=10, label='End')
        
        self.ax.set_xlabel('U', fontsize=12)
        self.ax.set_ylabel('V', fontsize=12)
        self.ax.set_title('Path Planning - UV Space', fontsize=14)
        self.ax.grid(True, alpha=0.3)
        self.ax.legend()
        self.ax.set_aspect('equal')
        
        self.get_logger().info('Path Visualizer initialized - waiting for path data...')
        
        # Start animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=500, blit=False)
        
    
    def path_callback(self, msg):
        """Callback for path data."""
        try:
            # Convert flat array to Nx2 array
            data = np.array(msg.data)
            if len(data) > 0:
                self.uv_path = data.reshape(-1, 2)
                self.updated = True
                self.get_logger().info(f'Received path with {len(self.uv_path)} points', once=True)
        except Exception as e:
            self.get_logger().error(f'Error processing path data: {e}')
    
    
    def update_plot(self, frame):
        """Update plot with new data."""
        if self.uv_path is not None and self.updated:
            # Update line
            self.line.set_data(self.uv_path[:, 0], self.uv_path[:, 1])
            
            # Update start/end markers
            self.start_point.set_data([self.uv_path[0, 0]], [self.uv_path[0, 1]])
            self.end_point.set_data([self.uv_path[-1, 0]], [self.uv_path[-1, 1]])
            
            # Auto-scale axes
            self.ax.relim()
            self.ax.autoscale_view()
            
            self.updated = False
        
        return self.line, self.start_point, self.end_point


def main(args=None):
    rclpy.init(args=args)
    
    visualizer = PathVisualizer()
    
    # Run ROS2 spin in a separate thread
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    spin_thread.start()
    
    try:
        # Show plot (blocking)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
