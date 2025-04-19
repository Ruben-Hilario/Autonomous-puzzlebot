import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, Point

class PIDBallFollower(Node):
    def __init__(self):
        super().__init__('PID_Ball_Tracker')
        self.declare_parameter('distance', 3.0)  # Desired distance in meters
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

        self.prev_error_distance = 0.0
        self.integral_distance = 0.0

        self.min_linear_speed = 0.0
        self.max_linear_speed = 0.4

        self.get_logger().info('Ball Follower PID Node Initialized')

    def calculate_radius_from_distance(self, distance):
        # Empirically derived constant
        k = 113.7
        return k / distance if distance != 0 else float('inf')  # Avoid division by zero

    def calculate_distance_from_radius(self, radius):
        k = 113.7
        return k / radius if radius > 0 else float('inf')  # Avoid division by zero

    def PID_loop(self, msg):
        ball_x = msg.x
        ball_radius = msg.z  # Observed ball radius in pixels

        # === Angular control (horizontal alignment) ===
        error_angular = self.image_center_x - ball_x
        self.integral_angular += error_angular
        derivative_angular = error_angular - self.prev_error_angular
        angular_output = (
            self.kp_angular * error_angular +
            self.ki_angular * self.integral_angular +
            self.kd_angular * derivative_angular
        )
        self.prev_error_angular = error_angular

        # === Linear control (distance from ball) ===
        distance = self.get_parameter('distance').get_parameter_value().double_value
        desired_radius = self.calculate_radius_from_distance(distance)
        current_distance = self.calculate_distance_from_radius(ball_radius)
        distance_error = distance - current_distance

        error_distance = desired_radius - ball_radius
        self.integral_distance += error_distance
        derivative_distance = error_distance - self.prev_error_distance
        linear_output = (
            self.kp_linear * error_distance +
            self.ki_linear * self.integral_distance +
            self.kd_linear * derivative_distance
        )
        self.prev_error_distance = error_distance

        # Clamp linear output
        linear_output = np.clip(linear_output, self.min_linear_speed, self.max_linear_speed)

        # Publish Twist command
        twist = Twist()
        twist.linear.x = linear_output
        twist.angular.z = angular_output
        self.publisher.publish(twist)

        # Debug output
        self.get_logger().info(
            f"Ball X: {ball_x:.1f}, Ball Radius: {ball_radius:.1f} px, "
            f"Current Distance: {current_distance:.2f} m, "
            f"Desired Distance: {distance:.2f} m, Distance Error: {distance_error:.2f} m, "
            f"Linear Output: {linear_output:.3f}, "
            f"Angular Error: {error_angular:.2f}, Angular Output: {angular_output:.3f}"
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
