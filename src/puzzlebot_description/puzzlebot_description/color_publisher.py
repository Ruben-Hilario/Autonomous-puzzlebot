#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ColorPublisher(Node):
    def __init__(self):
        super().__init__('color_command_publisher')
        self.publisher_ = self.create_publisher(String, '/color_command', 10)
        self.timer = self.create_timer(3.0, self.timer_callback)
        self.colors = ['red', 'green', 'yellow']
        self.index = 0

    def timer_callback(self):
        msg = String()
        msg.data = self.colors[self.index]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing color: {msg.data}')
        self.index = (self.index + 1) % len(self.colors)

def main(args=None):
    rclpy.init(args=args)
    node = ColorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()