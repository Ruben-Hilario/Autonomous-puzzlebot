import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Publisher to cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to pose
        self.create_subscription(Twist, 'pose', self.pose_callback, 10)

        # Declare parameters
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        self.state = 0  # 0: rotate, 1: move forward, 2: stop
        self.state_start_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def pose_callback(self, msg):
        # Extract the distance and rotation angle from the received message
        distance = msg.linear.x
        angle = msg.angular.z

        self.get_logger().info(f"Received path info: distance {distance}, angle {angle}")
        
        # Control logic
        cmd = Twist()

        # Rotate to face the goal
        if self.state == 0:
            cmd.angular.z = np.sign(angle) * self.angular_speed
            self.get_logger().info('Rotating to goal...')
            if abs(angle) <= 0.1:  # A small threshold to stop rotating
                self.state = 1
                self.state_start_time = self.get_clock().now()
                self.get_logger().info('Finished rotation. Moving forward...')
        
        elif self.state == 1:
            cmd.linear.x = self.linear_speed
            self.get_logger().info('Moving forward...')
            if distance <= 0.1:  # A small threshold to stop moving
                self.state = 2
                self.get_logger().info('Reached goal.')
        
        elif self.state == 2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Stopped.')

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
