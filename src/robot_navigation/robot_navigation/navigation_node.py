#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy import qos
import transforms3d

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Goal position
        self.goal_x = 3.0
        self.goal_y = 8.0
        
        # Navigation parameters
        self.safe_distance = 0.5  # meters
        self.max_linear_vel = 0.2
        self.max_angular_vel = 0.5
        self.position_threshold = 0.1  # meters
        self.angle_threshold = 0.1  # radians
        
        # State variables
        self.current_pose = None
        self.map_data = None
        self.map_info = None
        self.path = None
        self.current_path_index = 0
        self.state = "PLANNING"  # PLANNING, FOLLOWING, AVOIDING
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            qos.qos_profile_sensor_data
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos.qos_profile_sensor_data
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos.qos_profile_sensor_data
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            'planned_path',
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Navigation node initialized')
    
    def map_callback(self, msg):
        """Store map data"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info
    
    def odom_callback(self, msg):
        """Update current pose"""
        self.current_pose = msg.pose.pose
    
    def scan_callback(self, msg):
        """Process LiDAR data for obstacle avoidance"""
        if not self.ranges:
            return
            
        # Get minimum distance in front of robot
        front_ranges = msg.ranges[len(msg.ranges)//4:3*len(msg.ranges)//4]
        self.min_dist = min([r for r in front_ranges if r > 0.0])
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        if self.map_info is None:
            return None, None
            
        grid_x = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        grid_y = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        if self.map_info is None:
            return None, None
            
        x = grid_x * self.map_info.resolution + self.map_info.origin.position.x
        y = grid_y * self.map_info.resolution + self.map_info.origin.position.y
        return x, y
    
    def plan_path(self):
        """Plan path to goal using A* algorithm"""
        if self.map_data is None or self.current_pose is None:
            return
            
        # Convert start and goal to grid coordinates
        start_x, start_y = self.world_to_grid(
            self.current_pose.position.x,
            self.current_pose.position.y
        )
        goal_x, goal_y = self.world_to_grid(self.goal_x, self.goal_y)
        
        # Simple path planning (straight line for now)
        # In a real implementation, you would use A* or another path planning algorithm
        path = []
        current_x, current_y = start_x, start_y
        
        while abs(current_x - goal_x) > 1 or abs(current_y - goal_y) > 1:
            if abs(current_x - goal_x) > abs(current_y - goal_y):
                current_x += 1 if goal_x > current_x else -1
            else:
                current_y += 1 if goal_y > current_y else -1
                
            # Check if cell is free
            if self.map_data[current_y, current_x] > 50:  # Occupied
                continue
                
            path.append((current_x, current_y))
        
        # Convert path to world coordinates
        self.path = []
        for grid_x, grid_y in path:
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            pose = PoseStamped()
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            self.path.append(pose)
        
        # Publish path for visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = self.path
        self.path_pub.publish(path_msg)
        
        self.current_path_index = 0
        self.state = "FOLLOWING"
    
    def control_loop(self):
        """Main control loop"""
        if self.current_pose is None:
            return
            
        cmd = Twist()
        
        if self.state == "PLANNING":
            self.plan_path()
            return
            
        elif self.state == "FOLLOWING":
            if self.path is None or self.current_path_index >= len(self.path):
                self.state = "PLANNING"
                return
                
            # Get current goal from path
            current_goal = self.path[self.current_path_index]
            
            # Calculate distance and angle to goal
            dx = current_goal.pose.position.x - self.current_pose.position.x
            dy = current_goal.pose.position.y - self.current_pose.position.y
            distance = np.sqrt(dx*dx + dy*dy)
            
            # Get current orientation
            current_quat = [
                self.current_pose.orientation.w,
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z
            ]
            _, _, current_yaw = transforms3d.euler.quat2euler(current_quat)
            
            # Calculate target angle
            target_angle = np.arctan2(dy, dx)
            angle_error = self.wrap_to_pi(target_angle - current_yaw)
            
            # Check for obstacles
            if hasattr(self, 'min_dist') and self.min_dist < self.safe_distance:
                self.state = "AVOIDING"
                return
            
            # Move to goal
            if distance > self.position_threshold:
                # Proportional control for linear velocity
                cmd.linear.x = min(self.max_linear_vel, 0.5 * distance)
                
                # Proportional control for angular velocity
                cmd.angular.z = min(self.max_angular_vel, 1.0 * angle_error)
            else:
                # Reached current waypoint
                self.current_path_index += 1
                if self.current_path_index >= len(self.path):
                    self.state = "PLANNING"
        
        elif self.state == "AVOIDING":
            # Simple obstacle avoidance
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular_vel
            
            # Check if path is clear
            if hasattr(self, 'min_dist') and self.min_dist > self.safe_distance:
                self.state = "FOLLOWING"
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def wrap_to_pi(self, angle):
        """Wrap angle to [-π, π]"""
        return np.arctan2(np.sin(angle), np.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()