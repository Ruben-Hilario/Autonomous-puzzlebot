# Imports
# import rclpy
# from rclpy.node import Node
# import numpy as np
# from std_msgs.msg import Float32
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry
# import transforms3d
# from rclpy import qos

# class CustomMotionController(Node):
#     def __init__(self):
#         super().__init__('custom_motion_controller')

#         # Publisher to /cmd_vel
#         self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

#         # Subscriber to odometry
#         self.odom_sub = self.create_subscription(
#             Odometry,
#             'odom',
#             self.odom_callback,
#             qos.qos_profile_sensor_data
#         )

#         # Control variables
#         self.state = 0  # 0: avanzar 1.1, 1: girar izq 90, 2: avanzar 1.05, 3: girar der 90, 4: avanzar, 5: stop
#         self.current_pose = None
#         self.goal_pose = None
#         self.initial_yaw = None # Para calcular el giro relativo

#         # Motion parameters
#         self.forward_distance_1 = 1.1  # meters
#         self.forward_distance_2 = 1.05 # meters
#         self.rotation_angle = np.pi / 2  # 90 degrees in radians

#         # PID parameters (tune these as needed)
#         self.Kp_linear = 0.1
#         self.Ki_linear = 0.01
#         self.Kd_linear = 0.001

#         self.Kp_angular = 1.0
#         self.Ki_angular = 0.05
#         self.Kd_angular = 0.2

#         # Error accumulators for integral term
#         self.integral_linear = 0.0
#         self.integral_angular = 0.0
#         self.prev_error_linear = 0.0
#         self.prev_error_angular = 0.0

#         # Thresholds
#         self.position_threshold = 0.05  # 5 cm
#         self.angle_threshold = 0.05    # ~3 degrees

#         # Timer for control loop
#         self.timer_period = 0.1  # 10 Hz
#         self.timer = self.create_timer(self.timer_period, self.control_loop)

#         self.get_logger().info('Custom motion controller initialized!')

#     def odom_callback(self, msg):
#         """Store current robot pose from odometry"""
#         self.current_pose = msg.pose.pose

#         # Initialize goal pose and initial yaw at first odom message
#         if self.goal_pose is None and self.current_pose is not None:
#             self.set_new_goal()
#             orientation = self.current_pose.orientation
#             quat = [orientation.w, orientation.x, orientation.y, orientation.z]
#             _, _, self.initial_yaw = transforms3d.euler.quat2euler(quat)

#     def set_new_goal(self):
#         """Set the next goal pose based on current state"""
#         if self.current_pose is None:
#             return

#         self.goal_pose = PoseStamped()
#         self.goal_pose.header.stamp = self.get_clock().now().to_msg()
#         self.goal_pose.header.frame_id = 'odom'

#         # Get current orientation as Euler angles
#         orientation = self.current_pose.orientation
#         quat = [orientation.w, orientation.x, orientation.y, orientation.z]
#         _, _, current_yaw = transforms3d.euler.quat2euler(quat)

#         if self.state == 0:  # Move forward 1.1
#             self.goal_pose.pose.position.x = (
#                 self.current_pose.position.x +
#                 self.forward_distance_1 * np.cos(current_yaw)
#             )
#             self.goal_pose.pose.position.y = (
#                 self.current_pose.position.y +
#                 self.forward_distance_1 * np.sin(current_yaw)
#             )
#             self.goal_pose.pose.orientation = self.current_pose.orientation
#         elif self.state == 1:  # Turn left 90
#             self.goal_pose.pose.position = self.current_pose.position
#             new_yaw = current_yaw + self.rotation_angle
#             new_quat = transforms3d.euler.euler2quat(0, 0, new_yaw)
#             self.goal_pose.pose.orientation.x = new_quat[1]
#             self.goal_pose.pose.orientation.y = new_quat[2]
#             self.goal_pose.pose.orientation.z = new_quat[3]
#             self.goal_pose.pose.orientation.w = new_quat[0]
#         elif self.state == 2:  # Move forward 1.05
#             self.goal_pose.pose.position.x = (
#                 self.current_pose.position.x +
#                 self.forward_distance_2 * np.cos(current_yaw)
#             )
#             self.goal_pose.pose.position.y = (
#                 self.current_pose.position.y +
#                 self.forward_distance_2 * np.sin(current_yaw)
#             )
#             self.goal_pose.pose.orientation = self.current_pose.orientation
#         elif self.state == 3:  # Turn right 90
#             self.goal_pose.pose.position = self.current_pose.position
#             new_yaw = current_yaw - self.rotation_angle
#             new_quat = transforms3d.euler.euler2quat(0, 0, new_yaw)
#             self.goal_pose.pose.orientation.x = new_quat[1]
#             self.goal_pose.pose.orientation.y = new_quat[2]
#             self.goal_pose.pose.orientation.z = new_quat[3]
#             self.goal_pose.pose.orientation.w = new_quat[0]
#         elif self.state == 4: # Move forward indefinitely
#             self.goal_pose.pose.position.x = (
#                 self.current_pose.position.x + 1.0 * np.cos(current_yaw) # Small increment to keep moving
#             )
#             self.goal_pose.pose.position.y = (
#                 self.current_pose.position.y + 1.0 * np.sin(current_yaw) # Small increment to keep moving
#             )
#             self.goal_pose.pose.orientation = self.current_pose.orientation
#         elif self.state == 5: # Stop
#             pass # Goal pose not needed

#         self.get_logger().info(f'New goal set (state {self.state}): {self.goal_pose.pose.position if self.goal_pose else "None"}')

#     def calculate_errors(self):
#         """Calculate position and angle errors"""
#         if self.current_pose is None or self.goal_pose is None:
#             return None, None

#         # Position error (distance to goal)
#         dx = self.goal_pose.pose.position.x - self.current_pose.position.x
#         dy = self.goal_pose.pose.position.y - self.current_pose.position.y
#         distance_error = np.sqrt(dx**2 + dy**2)

#         # Angle error
#         current_quat = [
#             self.current_pose.orientation.w,
#             self.current_pose.orientation.x,
#             self.current_pose.orientation.y,
#             self.current_pose.orientation.z
#         ]
#         goal_quat = [
#             self.goal_pose.pose.orientation.w,
#             self.goal_pose.pose.orientation.x,
#             self.goal_pose.pose.orientation.y,
#             self.goal_pose.pose.orientation.z
#         ]

#         _, _, current_yaw = transforms3d.euler.quat2euler(current_quat)
#         _, _, goal_yaw = transforms3d.euler.quat2euler(goal_quat)
#         angle_error = self.wrap_to_Pi(goal_yaw - current_yaw)

#         return distance_error, angle_error

#     def control_loop(self):
#         if self.current_pose is None:
#             return

#         cmd = Twist()
#         distance_error, angle_error = self.calculate_errors()

#         if self.state < 4: # States with defined goals
#             if distance_error is None or angle_error is None:
#                 return

#             if self.state in [0, 2]: # Moving forward
#                 # PID for linear motion
#                 self.integral_linear += distance_error * self.timer_period
#                 derivative = (distance_error - self.prev_error_linear) / self.timer_period

#                 linear_vel = (
#                     self.Kp_linear * distance_error +
#                     self.Ki_linear * self.integral_linear +
#                     self.Kd_linear * derivative
#                 )

#                 # Limit velocity
#                 linear_vel = np.clip(linear_vel, -0.5, 0.5)
#                 cmd.linear.x = linear_vel

#                 # Small angular correction to face goal
#                 target_angle = np.arctan2(
#                     self.goal_pose.pose.position.y - self.current_pose.position.y,
#                     self.goal_pose.pose.position.x - self.current_pose.position.x
#                 )
#                 current_quat = [
#                     self.current_pose.orientation.w,
#                     self.current_pose.orientation.x,
#                     self.current_pose.orientation.y,
#                     self.current_pose.orientation.z
#                 ]
#                 _, _, current_yaw = transforms3d.euler.quat2euler(current_quat)
#                 angle_correction = self.wrap_to_Pi(target_angle - current_yaw)

#                 cmd.angular.z = 0.5 * angle_correction # Simple P control for heading

#                 # Check if reached position
#                 if distance_error < self.position_threshold:
#                     self.state += 1
#                     self.reset_pid()
#                     self.set_new_goal()
#                     self.get_logger().info(f'Reached forward target. Moving to state {self.state}...')

#             elif self.state in [1, 3]: # Turning
#                 # PID for angular motion
#                 self.integral_angular += angle_error * self.timer_period
#                 derivative = (angle_error - self.prev_error_angular) / self.timer_period

#                 angular_vel = (
#                     self.Kp_angular * angle_error +
#                     self.Ki_angular * self.integral_angular +
#                     self.Kd_angular * derivative
#                 )

#                 # Limit angular velocity
#                 angular_vel = np.clip(angular_vel, -1.0, 1.0)
#                 cmd.angular.z = angular_vel

#                 # Check if reached angle
#                 if abs(angle_error) < self.angle_threshold:
#                     self.state += 1
#                     self.reset_pid()
#                     self.set_new_goal()
#                     self.get_logger().info(f'Finished turn. Moving to state {self.state}...')

#         elif self.state == 4: # Move forward continuously
#             cmd.linear.x = 0.3 # Constant forward velocity
#             cmd.angular.z = 0.0
#             self.set_new_goal() # Keep updating the goal slightly ahead
#             self.get_logger().info('Moving forward continuously...')

#         elif self.state == 5: # Stop
#             cmd.linear.x = 0.0
#             cmd.angular.z = 0.0
#             self.timer.cancel()
#             self.get_logger().info('Stopping...')

#         # Store errors for next iteration
#         if distance_error is not None:
#             self.prev_error_linear = distance_error
#         if angle_error is not None:
#             self.prev_error_angular = angle_error

#         # Publish velocity command
#         self.cmd_vel_pub.publish(cmd)

#         # Transition to the next state if the current one is done (for states 4 to 5)
#         if self.state == 4:
#             # You might want to add a condition here to eventually stop
#             # For now, it will continue indefinitely until the program is stopped.
#             pass # Keep moving forward

#     def reset_pid(self):
#         """Reset PID accumulators when switching states"""
#         self.integral_linear = 0.0
#         self.integral_angular = 0.0
#         self.prev_error_linear = 0.0
#         self.prev_error_angular = 0.0

#     def wrap_to_Pi(self, theta):
#         """Wrap angle to [-π, π] range"""
#         result = np.fmod((theta + np.pi), (2 * np.pi))
#         if result < 0:
#             result += 2 * np.pi
#         return result - np.pi

# def main(args=None):
#     rclpy.init(args=args)
#     node = CustomMotionController()
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