# import rclpy
# from rclpy.node import Node
# import numpy as np
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry, Path
# import transforms3d
# from rclpy import qos

# class PathFollowerPID(Node):
#     def __init__(self):
#         super().__init__('path_follower_pid')

#         self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

#         # Suscripción a odometría
#         self.odom_sub = self.create_subscription(
#             Odometry,
#             'odom',
#             self.odom_callback,
#             qos.qos_profile_sensor_data
#         )

#         # Suscripción a la ruta planificada
#         self.path_sub = self.create_subscription(
#             Path,
#             '/planned_path',
#             self.path_callback,
#             10
#         )

#         self.current_pose = None
#         self.path = []  # Lista de PoseStamped
#         self.current_goal_index = 0

#         # Parámetros PID
#         self.Kp_linear = 0.2
#         self.Ki_linear = 0.0
#         self.Kd_linear = 0.01

#         self.Kp_angular = 1.0
#         self.Ki_angular = 0.0
#         self.Kd_angular = 0.1

#         self.integral_linear = 0.0
#         self.integral_angular = 0.0
#         self.prev_error_linear = 0.0
#         self.prev_error_angular = 0.0

#         self.position_threshold = 0.1  # metros
#         self.angle_threshold = 0.1  # radianes

#         self.timer_period = 0.1
#         self.timer = self.create_timer(self.timer_period, self.control_loop)

#         self.get_logger().info('Path follower PID initialized!')

#     def path_callback(self, msg: Path):
#         if len(msg.poses) == 0:
#             self.get_logger().warn('Ruta vacía recibida')
#             return

#         self.path = msg.poses
#         self.current_goal_index = 0
#         self.get_logger().info(f'Nueva ruta recibida con {len(self.path)} puntos')

#     def odom_callback(self, msg):
#         self.current_pose = msg.pose.pose

#     # def control_loop(self):
#     #     if self.current_pose is None or not self.path or self.current_goal_index >= len(self.path):
#     #         self.publish_stop()
#     #         return

#     #     goal_pose = self.path[self.current_goal_index].pose

#     #     # Calcular error de posición
#     #     dx = goal_pose.position.x - self.current_pose.position.x
#     #     dy = goal_pose.position.y - self.current_pose.position.y
#     #     distance_error = np.sqrt(dx*dx + dy*dy)

#     #     # Calcular error angular
#     #     current_quat = [
#     #         self.current_pose.orientation.w,
#     #         self.current_pose.orientation.x,
#     #         self.current_pose.orientation.y,
#     #         self.current_pose.orientation.z
#     #     ]
#     #     _, _, current_yaw = transforms3d.euler.quat2euler(current_quat)

#     #     target_yaw = np.arctan2(dy, dx)
#     #     angle_error = self.wrap_to_pi(target_yaw - current_yaw)

#     #     cmd = Twist()

#     #     # PID lineal
#     #     self.integral_linear += distance_error * self.timer_period
#     #     derivative_linear = (distance_error - self.prev_error_linear) / self.timer_period

#     #     linear_vel = (self.Kp_linear * distance_error +
#     #                   self.Ki_linear * self.integral_linear +
#     #                   self.Kd_linear * derivative_linear)
#     #     linear_vel = np.clip(linear_vel, 0.0, 0.3)

#     #     # PID angular
#     #     self.integral_angular += angle_error * self.timer_period
#     #     derivative_angular = (angle_error - self.prev_error_angular) / self.timer_period

#     #     angular_vel = (self.Kp_angular * angle_error +
#     #                    self.Ki_angular * self.integral_angular +
#     #                    self.Kd_angular * derivative_angular)
#     #     angular_vel = np.clip(angular_vel, -1.0, 1.0)

#     #     # Priorizar girar en sitio si el error angular es grande
#     #     if abs(angle_error) > 0.3:
#     #         linear_vel = 0.0

#     #     cmd.linear.x = linear_vel
#     #     cmd.angular.z = angular_vel

#     #     self.cmd_vel_pub.publish(cmd)

#     #     self.prev_error_linear = distance_error
#     #     self.prev_error_angular = angle_error

#     #     # Pasar al siguiente punto si se alcanzó el objetivo actual
#     #     if distance_error < self.position_threshold:
#     #         self.get_logger().info(f'Punto {self.current_goal_index} alcanzado')
#     #         self.current_goal_index += 1
#     #         self.reset_pid()
#     #         if self.current_goal_index >= len(self.path):
#     #             self.get_logger().info('Ruta completada, deteniendo robot')
#     #             self.publish_stop()
#     def control_loop(self):
#         if self.current_pose is None or not self.path or self.current_goal_index >= len(self.path):
#             self.publish_stop()
#             return

#         goal_pose = self.path[self.current_goal_index].pose

#         dx = goal_pose.position.x - self.current_pose.position.x
#         dy = goal_pose.position.y - self.current_pose.position.y
#         distance_error = np.sqrt(dx*dx + dy*dy)

#         current_quat = [
#             self.current_pose.orientation.w,
#             self.current_pose.orientation.x,
#             self.current_pose.orientation.y,
#             self.current_pose.orientation.z
#         ]
#         _, _, current_yaw = transforms3d.euler.quat2euler(current_quat)

#         target_yaw = np.arctan2(dy, dx)
#         angle_error = self.wrap_to_pi(target_yaw - current_yaw)

#         # Zona muerta angular para evitar "bailoteo"
#         if abs(angle_error) < 0.05:
#             angle_error = 0.0
#             self.integral_angular = 0.0  # Reset integral para anti-windup

#         cmd = Twist()

#         # PID lineal
#         self.integral_linear += distance_error * self.timer_period
#         derivative_linear = (distance_error - self.prev_error_linear) / self.timer_period

#         linear_vel = (self.Kp_linear * distance_error +
#                     self.Ki_linear * self.integral_linear +
#                     self.Kd_linear * derivative_linear)
#         linear_vel = np.clip(linear_vel, 0.0, 0.3)

#         # PID angular
#         self.integral_angular += angle_error * self.timer_period
#         derivative_angular = (angle_error - self.prev_error_angular) / self.timer_period

#         angular_vel = (self.Kp_angular * angle_error +
#                     self.Ki_angular * self.integral_angular +
#                     self.Kd_angular * derivative_angular)
#         angular_vel = np.clip(angular_vel, -1.0, 1.0)

#         # Si el error angular es grande, gira sin avanzar
#         if abs(angle_error) > 0.5:
#             linear_vel = 0.0

#         cmd.linear.x = linear_vel
#         cmd.angular.z = angular_vel

#         self.cmd_vel_pub.publish(cmd)

#         self.get_logger().info(f'Dist error: {distance_error:.3f}, Angle error: {angle_error:.3f}, Lin vel: {linear_vel:.3f}, Ang vel: {angular_vel:.3f}')

#         self.prev_error_linear = distance_error
#         self.prev_error_angular = angle_error

#         if distance_error < self.position_threshold:
#             self.get_logger().info(f'Punto {self.current_goal_index} alcanzado')
#             self.current_goal_index += 1
#             self.reset_pid()
#             if self.current_goal_index >= len(self.path):
#                 self.get_logger().info('Ruta completada, deteniendo robot')
#                 self.publish_stop()


#     def publish_stop(self):
#         stop_cmd = Twist()
#         stop_cmd.linear.x = 0.0
#         stop_cmd.angular.z = 0.0
#         self.cmd_vel_pub.publish(stop_cmd)

#     def reset_pid(self):
#         self.integral_linear = 0.0
#         self.integral_angular = 0.0
#         self.prev_error_linear = 0.0
#         self.prev_error_angular = 0.0

#     def wrap_to_pi(self, angle):
#         return (angle + np.pi) % (2 * np.pi) - np.pi

# def main(args=None):
#     rclpy.init(args=args)
#     node = PathFollowerPID()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if rclpy.ok():
#             rclpy.shutdown()
#         node.destroy_node()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.subscription = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_path = []
        self.current_index = 0

        # Parámetros de control (ajustar según tu robot)
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 0.5  # rad/s
        self.goal_tolerance = 0.05  # metros

        # Estado del robot (debes actualizarlo desde odometría o tf)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Timer para control periódico
        self.timer = self.create_timer(0.1, self.control_loop)

        # Suscríbete a odometría o tf para actualizar la pose real del robot aquí
        # Ejemplo con tf o nav_msgs/Odometry (no incluido por simplicidad)

    def path_callback(self, msg):
        self.get_logger().info(f'Recibido path con {len(msg.poses)} puntos')
        self.current_path = msg.poses
        self.current_index = 0

    def control_loop(self):
        if not self.current_path or self.current_index >= len(self.current_path):
            # No path o fin del path
            self.stop_robot()
            return

        # Lee el objetivo actual en el path
        goal_pose = self.current_path[self.current_index].pose.position

        # Calcula la distancia al objetivo
        dx = goal_pose.x - self.robot_x
        dy = goal_pose.y - self.robot_y
        distance = math.sqrt(dx*dx + dy*dy)

        # Calcula el ángulo al objetivo
        goal_yaw = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(goal_yaw - self.robot_yaw)

        twist = Twist()

        if distance < self.goal_tolerance:
            # Avanzar al siguiente punto del path
            self.get_logger().info(f'Punto {self.current_index} alcanzado.')
            self.current_index += 1
            self.stop_robot()
        else:
            # Control simple proporcional
            if abs(angle_diff) > 0.1:
                twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                twist.linear.x = 0.0
            else:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0

            self.cmd_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
