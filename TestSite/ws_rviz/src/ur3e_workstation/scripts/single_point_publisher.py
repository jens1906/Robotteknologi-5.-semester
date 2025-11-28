#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class SinglePointPublisher(Node):
    def __init__(self):
        super().__init__('single_point_publisher')
        self.pub = self.create_publisher(Float64MultiArray, '/tool_orientation/xyz_rotation', 10)
        self.timer = self.create_timer(0.5, self.publish_once)
        self.published = False
        self.get_logger().info('SinglePointPublisher started')

    def publish_once(self):
        if self.published:
            return
        msg = Float64MultiArray()
        # Pose: [x, y, z, qx, qy, qz, qw]
        msg.data = [0.05, -0.218, 0.05, 0.0, 0.0, 0.0, 1.0]
        self.pub.publish(msg)
        self.get_logger().info('Published single point: x=0.05, y=-0.218, z=0.05')
        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = SinglePointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
