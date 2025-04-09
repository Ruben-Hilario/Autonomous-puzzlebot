# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class SquareMovement(Node):
    def __init__(self):
        super().__init__('square_movement')

        #self.wait_for_ros_time()

        # Publisher to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Time-based control variables
        self.state = 0  # 0-3: forward + turn states, 4: stop
        self.state_start_time = self.get_clock().now()
        self.side_count = 0  # Track how many sides we've completed

        # Define speeds
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s

        # Define durations (seconds)
        self.forward_time = 2.0 / self.linear_speed   # Time to move 2m per side
        self.turn_time = (np.pi/2.1) / self.angular_speed  # Time to rotate 90 deg (π/2 rad)

        # Timer to update state machine
        self.timer_period = 0.1  # 10 Hz control loop
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Square movement controller initialized!')
        
    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9

        cmd = Twist()

        # State machine for square movement
        if self.state == 0:
            # Move forward for one side
            cmd.linear.x = self.linear_speed
            self.get_logger().info('Moving forward...')
            if elapsed_time >= self.forward_time:
                self.state = 1
                self.state_start_time = now
                self.side_count += 1
                self.get_logger().info(f'Finished side {self.side_count}. Starting turn...')

        elif self.state == 1:
            # Turn 90 degrees
            cmd.angular.z = self.angular_speed
            self.get_logger().info('Turning 90 degrees...')
            if elapsed_time >= self.turn_time:
                if self.side_count < 4:  # Only 4 sides in a square
                    self.state = 0  # Back to forward movement
                    self.state_start_time = now
                    self.get_logger().info('Finished turn. Moving forward...')
                else:
                    self.state = 2  # All sides completed
                    self.state_start_time = now
                    self.get_logger().info('Square completed! Stopping...')

        elif self.state == 2:
            # Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Stopped.')
            # Optionally: cancel the timer after stopping
            self.timer.cancel()

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)

    def wrap_to_Pi(self, theta):
        result = np.fmod((theta+np.pi),(2*np.pi))
        if (result<0):
            result += 2 * np.pi
        return result - np.pi

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'ROS time is active! Start time: {now.nanoseconds * 1e-9:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    node = SquareMovement()
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