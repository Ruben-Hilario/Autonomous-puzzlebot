import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class BallSineMover(Node):
    def __init__(self):
        super().__init__('ball_sine_mover')

        self.amplitude = 1.5
        self.frequency = 0.3
        self.forward_speed = 0.2

        self.start_time = self.get_clock().now()

        # 👇 Publisher to the Gazebo cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/model/ball/cmd_vel',  # << use your world name here
            10
        )

        self.timer = self.create_timer(0.05, self.publish_velocity)

    def publish_velocity(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        cmd = Twist()
        cmd.linear.x = self.forward_speed
        cmd.linear.y = 2 * np.pi * self.frequency * self.amplitude * np.cos(2 * np.pi * self.frequency * t)
        cmd.linear.z = 0.0

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BallSineMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
