# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy import qos

class ColorControl(Node):
    def __init__(self):
        super().__init__('decision_node')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Float32,'state',self.state_cb, 10)
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_cb,qos.qos_profile_sensor_data)
        
        # Time-based control variables
        self.state = 0
        self.current_pose = None
        self.goal_pose = None
        self.state_start_time = self.get_clock().now()

        # Define speeds
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # === Linear PID constants ===
        self.kp_linear = 0.05
        self.ki_linear = 0.001
        self.kd_linear = 0.01

        self.prev_error_distance = 0.0
        self.integral_distance = 0.0

        self.min_linear_speed = 0.2
        #self.max_linear_speed = 0.4

        # Timer to update state machine
        self.timer_period = 0.1  # 10 Hz control loop
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info("Control Node Started")

    def state_cb(self,msg):
        self.state = msg.data
        self.get_logger().info(f"Current state: {self.state}")

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose
        if self.current_pose is None:
            self.get_logger().warn("Error not current pose detected")
            return

    def control_loop(self):
        cmd = Twist()
        
        # State machine for square movement
        if self.state == 0:
            #Green color -> Continue moving
            cmd.linear.x = self.linear_speed
            self.get_logger().info('Green Color Detected. Moving forward...')

        elif self.state == 1:
            #Yellow Color -> Slow
            cmd.linear.x = self.min_linear_speed
            self.get_logger().info('Yellow Color detected. Slowing Down...')

        elif self.state == 2:
            cmd.linear.x = 0.0
            self.get_logger().info('Red Color detected. Stopped.')

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ColorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()