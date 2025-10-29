import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RealsensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        self.publisher_ = self.create_publisher(String, 'camera_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('RealsensePublisher started.')

    def timer_callback(self):
        msg = String()
        msg.data = "Fake camera frame"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = RealsensePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
