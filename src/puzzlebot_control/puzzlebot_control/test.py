#Square PID
# square_pid_movement.py

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output


class SquareMovement(Node):
    def __init__(self):
        super().__init__('square_movement')

        # Publisher and Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.linear_error_pub = self.create_publisher(Float32, 'linear_error', 10)
        self.angular_error_pub = self.create_publisher(Float32, 'angular_error', 10)

        # Control state
        self.state = 0
        self.state_start_time = self.get_clock().now()
        self.side_count = 0

        # Control and geometry
        self.target_distance = 2.0  # meters
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Pose tracking
        self.current_yaw = 0.0
        self.current_position = (0.0, 0.0)
        self.start_position = None
        self.target_yaw = None

        # PID Controllers
        self.linear_pid = PID()
        self.angular_pid = PID()

        # Declare parameters for tuning
        self.declare_parameter('linear_Kp', 1.2)
        self.declare_parameter('linear_Ki', 0.0)
        self.declare_parameter('linear_Kd', 0.05)

        self.declare_parameter('angular_Kp', 1.5)
        self.declare_parameter('angular_Ki', 0.0)
        self.declare_parameter('angular_Kd', 0.1)

        self.update_pid_gains()
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Optional: log errors to file
        self.error_log = open('/tmp/pid_errors.csv', 'w')
        self.error_log.write('time,linear_error,angular_error\n')

        self.get_logger().info('Square movement PID controller initialized!')

    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9
        cmd = Twist()
        dt = self.timer_period
        time_now = now.nanoseconds * 1e-9

        linear_error = 0.0
        angular_error = 0.0

        if self.state == 0:
            if self.start_position is None:
                self.start_position = self.current_position

            distance_traveled = self.compute_distance(self.start_position, self.current_position)
            error = self.target_distance - distance_traveled
            linear_error = error

            linear_x = self.linear_pid.compute(error, dt)
            cmd.linear.x = max(min(linear_x, 0.4), 0.0)

            self.get_logger().info(f'Moving forward... Distance: {distance_traveled:.2f}, Error: {error:.2f}, Output: {linear_x:.2f}')

            if abs(error) < 0.03:
                self.state = 1
                self.state_start_time = now
                self.side_count += 1
                self.start_position = None
                self.get_logger().info(f'Finished side {self.side_count}. Starting turn...')

        elif self.state == 1:
            if self.target_yaw is None:
                self.target_yaw = self.wrap_to_Pi(self.current_yaw + np.pi / 2)

            error = self.wrap_to_Pi(self.target_yaw - self.current_yaw)
            angular_error = error

            angular_z = self.angular_pid.compute(error, dt)
            cmd.angular.z = max(min(angular_z, 1.0), -1.0)

            self.get_logger().info(f'Turning... Error: {error:.3f}, Output: {angular_z:.3f}')

            if abs(error) < 0.05:
                if self.side_count < 4:
                    self.state = 0
                    self.state_start_time = now
                    self.target_yaw = None
                    self.get_logger().info('Turn complete. Moving forward...')
                else:
                    self.state = 2
                    self.state_start_time = now
                    self.get_logger().info('Square completed! Stopping...')

        elif self.state == 2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Stopped.')
            self.timer.cancel()

        # Publish
        self.cmd_vel_pub.publish(cmd)
        self.linear_error_pub.publish(Float32(data=linear_error))
        self.angular_error_pub.publish(Float32(data=angular_error))
        self.error_log.write(f'{time_now},{linear_error},{angular_error}\n')

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y**2 + orientation_q.z**2)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        pos = msg.pose.pose.position
        self.current_position = (pos.x, pos.y)

    def compute_distance(self, start_pos, current_pos):
        dx = current_pos[0] - start_pos[0]
        dy = current_pos[1] - start_pos[1]
        return math.sqrt(dx ** 2 + dy ** 2)

    def wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi), (2 * np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi

    def update_pid_gains(self):
        lin_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        lin_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        lin_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value

        ang_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        ang_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        ang_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value

        self.linear_pid.Kp = lin_Kp
        self.linear_pid.Ki = lin_Ki
        self.linear_pid.Kd = lin_Kd

        self.angular_pid.Kp = ang_Kp
        self.angular_pid.Ki = ang_Ki
        self.angular_pid.Kd = ang_Kd

        self.get_logger().info(f'Updated PID gains. Linear: ({lin_Kp}, {lin_Ki}, {lin_Kd}), Angular: ({ang_Kp}, {ang_Ki}, {ang_Kd})')

    def parameter_callback(self, params):
        self.update_pid_gains()
        return rclpy.parameter.ParameterEventHandler.Result(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = SquareMovement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'error_log'):
            node.error_log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
