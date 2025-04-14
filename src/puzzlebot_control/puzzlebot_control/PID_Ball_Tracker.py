import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist,Point

class PIDBallFollower(Node):
    def __init__(self):
        super().__init__('PID_Ball_Tracker')
        self.subscription = self.create_subscription(Point, 'ball_pos', self.PID_loop, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Image center assumed (adjust if needed)
        self.image_width = 640  # Default Gazebo cam width, change if different
        self.image_center_x = self.image_width / 2.0

        # PID constants (tune as needed)
        self.kp = 0.005
        self.ki = 0.0001
        self.kd = 0

        self.prev_error = 0.0
        self.integral = 0.0

        self.get_logger().info('Ball Follower PID Node Initialized')
        
    def PID_loop(self,msg):
        #logic
        ball_x = msg.x
        error = self.image_center_x - ball_x  # Positive if ball is to the left
        # PID calculations
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Save error for next iteration
        self.prev_error = error

        # Create Twist message
        twist = Twist()
        twist.linear.x = 0.1  # Constant forward speed
        twist.angular.z = output  # PID-controlled turn

        self.publisher.publish(twist)
        self.get_logger().info(f"Error: {error:.2f}, Angular Z: {output:.4f}")

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

