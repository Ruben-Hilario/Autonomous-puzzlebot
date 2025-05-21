import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, Point

class LineFollower(Node):
    def __init__(self):
        super().__init__('PID_Line_Follower')
        self.subscription = self.create_subscription(Point, 'line_offset', self.PID_loop, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Image parameters
        self.image_width = 640
        self.image_center_x = self.image_width/2.0

        # === Angular PID constants ===
        self.kp_angular = 0.005
        self.ki_angular = 0.0001
        self.kd_angular = 0.0

        self.prev_error_angular = 0.0
        self.integral_angular = 0.0
        self.linear_speed = 0.2

        self.get_logger().info('Ball Follower PID Node Initialized')

    def PID_loop(self, msg):
        offset = msg.x
        # === Angular control (horizontal alignment) ===
        error_angular = self.image_center_x - offset
        self.integral_angular += error_angular
        derivative_angular = error_angular - self.prev_error_angular
        angular_output = (
            self.kp_angular * error_angular +
            self.ki_angular * self.integral_angular +
            self.kd_angular * derivative_angular
        )
        self.prev_error_angular = error_angular

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = angular_output
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
