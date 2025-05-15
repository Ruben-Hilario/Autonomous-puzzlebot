import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rclpy import qos
import transforms3d

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, qos.qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.ranges = None
        self.min_dist = float('inf')
        self.safe_distance = 0.5
        self.current_pose = None
        self.initial_yaw = None
        self.target_yaw = None
        self.turn_done = False

        self.IDLE = 0
        self.TURNING = 1
        self.DONE_TURNING = 2
        self.state = self.IDLE

        self.Kp = 1.5
        self.Ki = 0.01
        self.Kd = 0.1
        self.integral = 0.0
        self.prev_error = 0.0
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def lidar_cb(self, msg):
        self.ranges = [r for r in msg.ranges if r > 0.0]
        if self.ranges:
            self.min_dist = min(self.ranges)

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def get_yaw(self):
        if not self.current_pose:
            return None
        q = self.current_pose.orientation
        quat = [q.w, q.x, q.y, q.z]
        _, _, yaw = transforms3d.euler.quat2euler(quat)
        return yaw

    def control_loop(self):
        if self.current_pose is None:
            return

        cmd = Twist()
        current_yaw = self.get_yaw()

        if self.state == self.IDLE:
            if self.min_dist < self.safe_distance and not self.turn_done:
                self.get_logger().info('Obstacle detected: Starting 90-degree turn.')
                self.state = self.TURNING
                self.initial_yaw = current_yaw
                self.target_yaw = self.normalize_angle(self.initial_yaw + np.pi / 2)
                self.integral = 0.0
                self.prev_error = 0.0
            else:
                cmd.linear.x = 0.2

        elif self.state == self.TURNING:
            error = self.normalize_angle(self.target_yaw - current_yaw)
            self.integral += error * self.timer_period
            derivative = (error - self.prev_error) / self.timer_period

            angular_z = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            angular_z = np.clip(angular_z, -1.0, 1.0)
            cmd.angular.z = angular_z
            self.prev_error = error

            if abs(error) < 0.05:
                self.get_logger().info('Turn completed.')
                self.state = self.DONE_TURNING
                self.turn_done = True

        elif self.state == self.DONE_TURNING:
            cmd.linear.x = 0.2

        self.pub.publish(cmd)

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
