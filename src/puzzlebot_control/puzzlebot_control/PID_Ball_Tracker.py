import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, Point

class PIDBallFollower(Node):
    def __init__(self):
        super().__init__('PID_Ball_Tracker')
        self.declare_parameter('desired_radius', 20.0)
        self.subscription = self.create_subscription(Point, 'ball_pos', self.PID_loop, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Image parameters
        self.image_width = 1280
        self.image_center_x = self.image_width / 2.0

        # === Angular PID constants ===
        self.kp_angular = 0.005
        self.ki_angular = 0.0001
        self.kd_angular = 0.0

        self.prev_error_angular = 0.0
        self.integral_angular = 0.0

        # === Linear PID constants ===
        self.kp_linear = 0.05
        self.ki_linear = 0.001
        self.kd_linear = 0.01

        self.prev_error_radius = 0.0
        self.integral_radius = 0.0

        self.min_linear_speed = 0.0
        self.max_linear_speed = 0.4

        self.get_logger().info('Ball Follower PID Node Initialized')

    def PID_loop(self, msg):
        ball_x = msg.x
        ball_radius = msg.z 

        # === Angular control (X alignment) ===
        error_angular = self.image_center_x - ball_x
        self.integral_angular += error_angular
        derivative_angular = error_angular - self.prev_error_angular
        angular_output = (
            self.kp_angular * error_angular +
            self.ki_angular * self.integral_angular +
            self.kd_angular * derivative_angular
        )
        self.prev_error_angular = error_angular

        # === Linear control (radius distance) ===
        desired_radius = self.get_parameter('desired_radius').get_parameter_value().double_value
        error_radius = desired_radius - ball_radius
        self.integral_radius += error_radius
        derivative_radius = error_radius - self.prev_error_radius
        linear_output = (
            self.kp_linear * error_radius +
            self.ki_linear * self.integral_radius +
            self.kd_linear * derivative_radius
        )
        self.prev_error_radius = error_radius

        # Clamp linear output to avoid spikes
        linear_output = np.clip(linear_output, self.min_linear_speed, self.max_linear_speed)

        # Publish Twist message
        twist = Twist()
        twist.linear.x = linear_output
        twist.angular.z = angular_output
        self.publisher.publish(twist)

        # Debug logging
        self.get_logger().info(
            f"Ball X: {ball_x:.1f}, Radius: {ball_radius:.1f}, "
            f"Desired Radius: {desired_radius:.1f}, Radius Error: {error_radius:.2f}, "
            f"Linear X: {linear_output:.3f}, Angular Error: {error_angular:.2f}, Angular Z: {angular_output:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PIDBallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
