#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rclpy import qos
import transforms3d
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Subscribers
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_cb, qos.qos_profile_sensor_data)
        
        # Publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Navigation parameters
        self.goal_x = 3.0
        self.goal_y = 8.0
        self.safe_distance = 0.5 # meters
        self.frontal_range = 40  # degrees on each side (60° total frontal view)
        self.max_linear_vel = 0.2
        self.max_angular_vel = 0.5
        self.position_threshold = 0.1  # meters
        self.angle_threshold = 0.1  # radians
        
        # State variables
        self.current_pose = None
        self.ranges = None
        self.angle_increment = 0.0
        self.angle_min = 0.0
        self.min_dist = float('inf')
        
        # Navigation states
        self.state = "NAVIGATING"  # NAVIGATING, AVOIDING, TURNING
        self.turn_direction = 1  # 1 for right, -1 for left (alternates)
        self.turn_angle = math.pi/2  # 90 degrees in radians
        self.start_turn_pose = None
        self.angle_turned = 0.0
        
        # Control parameters
        self.Kp_linear = 0.1
        self.Kp_angular = 0.1
        
        # Timer for control loop
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info('Navigation node initialized')
    
    def odom_cb(self, msg):
        """Update current pose and calculate turn angle"""
        prev_pose = self.current_pose
        self.current_pose = msg.pose.pose
        
        # Calculate angle turned since start of turn
        if self.state == "TURNING" and self.start_turn_pose and prev_pose:
            current_quat = [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ]
            start_quat = [
                self.start_turn_pose.orientation.x,
                self.start_turn_pose.orientation.y,
                self.start_turn_pose.orientation.z,
                self.start_turn_pose.orientation.w
            ]
            
            _, _, current_yaw = transforms3d.euler.quat2euler(current_quat)
            _, _, start_yaw = transforms3d.euler.quat2euler(start_quat)
            self.angle_turned = abs(self.wrap_to_pi(current_yaw - start_yaw))
    
    def lidar_cb(self, msg):
        """Process LiDAR data for obstacle detection"""
        self.ranges = msg.ranges
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min
        
        if self.ranges:
            num_ranges = len(self.ranges)
            self.min_dist = float('inf')
            
            # Convert angles to indices
            front_angle = 0.0
            center_idx = int((front_angle - self.angle_min) / self.angle_increment)

            range_start = center_idx - int(self.frontal_range * math.pi / 180 / self.angle_increment)
            range_end = center_idx + int(self.frontal_range * math.pi / 180 / self.angle_increment)
            
            # Ensure indices are within range
            range_start = max(0, range_start)
            range_end = min(num_ranges - 1, range_end)
            
            # Find minimum distance in frontal range
            for i in range(range_start, range_end):
                if 0 < self.ranges[i] < self.min_dist:
                    self.min_dist = self.ranges[i]
            
            # Reset min_dist if no obstacles in frontal range
            if self.min_dist == float('inf'):
                self.min_dist = None
    
    def is_obstacle_in_front(self):
        """Check if there's an obstacle in the frontal range"""
        return self.min_dist is not None and self.min_dist < self.safe_distance
    
    def calculate_goal_angle(self):
        """Calculate angle to goal"""
        if self.current_pose is None:
            return 0.0
            
        dx = self.goal_x - self.current_pose.position.x
        dy = self.goal_y - self.current_pose.position.y
        return math.atan2(dy, dx)
    
    def calculate_distance_to_goal(self):
        """Calculate distance to goal"""
        if self.current_pose is None:
            return float('inf')
            
        dx = self.goal_x - self.current_pose.position.x
        dy = self.goal_y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def control_loop(self):
        """Main control loop"""
        if self.current_pose is None:
            return
            
        cmd = Twist()
        
        if self.state == "NAVIGATING":
            # Check for obstacles
            if self.is_obstacle_in_front():
                self.get_logger().info(f'Obstacle detected at {self.min_dist:.2f} m - avoiding')
                self.state = "TURNING"
                self.start_turn_pose = self.current_pose
                self.angle_turned = 0.0
                self.turn_direction *= -1  # Alternate turn direction
                return
            
            # Calculate distance and angle to goal
            distance = self.calculate_distance_to_goal()
            goal_angle = self.calculate_goal_angle()
            
            # Get current orientation
            current_quat = [
                self.current_pose.orientation.w,
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z
            ]
            _, _, current_yaw = transforms3d.euler.quat2euler(current_quat)
            
            # Calculate angle error
            angle_error = self.wrap_to_pi(goal_angle - current_yaw)
            
            # Check if we've reached the goal
            if distance < self.position_threshold:
                self.get_logger().info('Goal reached!')
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                # Proportional control for linear velocity
                cmd.linear.x = min(self.max_linear_vel, self.Kp_linear * distance)
                
                # Proportional control for angular velocity
                cmd.angular.z = min(self.max_angular_vel, self.Kp_angular * angle_error)
        
        elif self.state == "TURNING":
            # Turn until clear of obstacle
            if self.angle_turned < self.turn_angle:
                cmd.linear.x = 0.0
                cmd.angular.z = self.max_angular_vel * self.turn_direction
            else:
                # Check if path is clear
                if not self.is_obstacle_in_front():
                    self.get_logger().info('Path clear - resuming navigation')
                    self.state = "NAVIGATING"
                else:
                    # Turn more if still blocked
                    self.angle_turned = 0.0
                    self.start_turn_pose = self.current_pose
        
        # Publish command
        self.pub_cmd.publish(cmd)
    
    def wrap_to_pi(self, angle):
        """Wrap angle to [-π, π]"""
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
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