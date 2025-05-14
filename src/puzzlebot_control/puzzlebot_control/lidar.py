import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
from rclpy import qos
import transforms3d

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.sub = self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)
        self.sub = self.create_subscription(Odom,'odom',self.odom_cb,qos.qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ranges = None  #LiDAR
        self.safe_distance = 0.4  # meters
        self.current_pose = None
        self.state = 0
        self.side_count = 0
        self.min_dist = None
        #self.goal_pose = None
        self.Kp_linear = 0.1
        self.Ki_linear = 0.01
        self.Kd_linear = 0.001
        self.Kp_ang = 1.0
        self.Kd_ang = 0.05
        self.Ki_linear = 0.2
        self.integral_linear = 0.0
        self.integral_ang = 0.0
        self.prev_error_linear = 0.0
        self.prev_error_ang = 0.0
        self.position_threshold = 0.05
        self.angle_threshold = 0.05
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period,self.control_loop)
        
    def odom_cb(self,msg):
        self.current_pose = msg.pose.pose
        # if self.goal_pose is None and self.current_pose is not None:
        #     self.set_new_goal()

    # def set_new_goal(self):
    #     if self.current_pose is None:
    #         return
    #     self.goal_pose = PoseStamped()
    #     self.goal_pose.header.stamp 0 self.get_clock().now().to_msg()
    #     self.goal_pose.heaer.frame_id='odom'
    #     orientation = self.current_pose.orientation
    #     quat = [orientation.w,orientation.x,orientation.y,orientation.z]
    #     _,_,current_yaw = transforms3d.euler.quat2euler(quat)
    #     if self.state == 0:  # Move forward
    #         self.goal_pose.pose.position.x = (
    #             self.current_pose.position.x + 
    #             self.side_length * np.cos(current_yaw)
    #         )
    #         self.goal_pose.pose.position.y = (
    #             self.current_pose.position.y + 
    #             self.side_length * np.sin(current_yaw)
    #         )
    #         self.goal_pose.pose.orientation = self.current_pose.orientation
    #     else:  # Turn
    #         self.goal_pose.pose.position = self.current_pose.position
    #         # Calculate new orientation after turn
    #         new_yaw = current_yaw + self.rotation_angle
    #         new_quat = transforms3d.euler.euler2quat(0, 0, new_yaw)
    #         self.goal_pose.pose.orientation.x = new_quat[1]
    #         self.goal_pose.pose.orientation.y = new_quat[2]
    #         self.goal_pose.pose.orientation.z = new_quat[3]
    #         self.goal_pose.pose.orientation.w = new_quat[0]
    #     self.get_logger().info(f'New goal set: {self.goal_pose.pose.position}')
        
    def lidar_cb(self, msg):
        self.ranges = [r for r in msg.ranges if r > 0.0]
        if not self.ranges:
            return
        self.min_dist = min(self.ranges)
            
    def calc_errors(self):
        """Calculate position and angle errors"""
        # if self.current_pose is None or self.goal_pose is None:
        #     return None, None

        # Position error (distance to goal)
        dx = self.goal_pose.pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.position.y
        distance_error = np.sqrt(dx**2 + dy**2)

        # Angle error
        current_quat = [
            self.current_pose.orientation.w,
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z
        ]
        goal_quat = [
            self.goal_pose.pose.orientation.w,
            self.goal_pose.pose.orientation.x,
            self.goal_pose.pose.orientation.y,
            self.goal_pose.pose.orientation.z
        ]
        
        _, _, current_yaw = transforms3d.euler.quat2euler(current_quat)
        _, _, goal_yaw = transforms3d.euler.quat2euler(goal_quat)
        angle_error = self.wrap_to_Pi(goal_yaw - current_yaw)

        return distance_error, angle_error
            
    def control_loop():
        cmd = Twist()
        if self.min_dist < self.safe_distance:
            self.get_logger().info(f'Obstacle detected at {self.min_dist:.2f} m - turning')
            cmd.angular.z = 0.5
        else:
            cmd.linear.x = 0.2

        self.pub.publish(cmd)
        
    def reset_pid(self):
        """Reset PID accumulators when switching states"""
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0

    def wrap_to_Pi(self, theta):
        """Wrap angle to [-π, π] range"""
        result = np.fmod((theta + np.pi), (2 * np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__=='__main__':
    main()
            

