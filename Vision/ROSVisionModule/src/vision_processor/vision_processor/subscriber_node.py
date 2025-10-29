import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        self.subscription = self.create_subscription(
            String, 'camera_data', self.listener_callback, 10)
        self.get_logger().info('VisionProcessor started and listening.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
