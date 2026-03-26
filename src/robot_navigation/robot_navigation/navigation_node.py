import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
from rclpy import qos
import transforms3d
import math

class SLAMNavigator(Node):
    def __init__(self):
        super().__init__('slam_navigator')
        
        # Subscribers
        self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, qos.qos_profile_sensor_data)
        
        # Publishers
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Map parameters
        self.map_resolution = 0.05  # 5cm per cell
        self.map_size = 200  # 10m x 10m map (increased size)
        self.map_origin_x = -5.0  # Center the map
        self.map_origin_y = -5.0
        self.map = np.zeros((self.map_size, self.map_size), dtype=np.int8)
        
        # Navigation parameters
        self.goal_x = 7.0
        self.goal_y = 8.0
        self.safe_distance = 0.2
        self.frontal_range = 60  # Increased frontal range for better detection
        self.max_linear_vel = 0.2
        self.max_angular_vel = 0.5
        self.position_threshold = 0.1  # meters
        self.angle_threshold = 0.05  # radians (about 3 degrees)
        
        # State variables
        self.ranges = None
        self.angle_increment = 0.0
        self.angle_min = 0.0
        self.current_pose = None
        self.min_dist = float('inf')
        
        # Navigation states
        self.state = "NAVIGATING"  # NAVIGATING, AVOIDING, TURNING
        self.turn_direction = 1
        self.turn_angle = math.pi/2
        self.start_turn_pose = None
        self.angle_turned = 0.0
        
        # Control parameters
        self.Kp_linear = 0.1
        self.Kp_angular = 0.1
        
        # Timer for control loop
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info('SLAM Navigator initialized')

    def lidar_cb(self, msg):
        """Process LiDAR data and update map"""
        self.ranges = msg.ranges
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min
        
        if self.ranges and self.current_pose:
            # Update map with new scan
            self.update_map()
            
            # Find minimum distance in frontal range
            num_ranges = len(self.ranges)
            front_angle = 0.0
            center_idx = int((front_angle - self.angle_min) / self.angle_increment)
            range_start = center_idx - int(self.frontal_range * math.pi / 180 / self.angle_increment)
            range_end = center_idx + int(self.frontal_range * math.pi / 180 / self.angle_increment)
            
            range_start = max(0, range_start)
            range_end = min(num_ranges - 1, range_end)
            
            self.min_dist = float('inf')
            for i in range(range_start, range_end):
                if 0 < self.ranges[i] < self.min_dist:
                    self.min_dist = self.ranges[i]
            
            if self.min_dist == float('inf'):
                self.min_dist = None

    def update_map(self):
        """Update occupancy grid map with new LiDAR data"""
        if not self.ranges or not self.current_pose:
            return
            
        # Get robot position in map coordinates
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Get robot orientation
        q = self.current_pose.orientation
        quat = [q.w, q.x, q.y, q.z]
        _, _, robot_yaw = transforms3d.euler.quat2euler(quat)
        
        # Update map for each LiDAR reading
        for i, range_val in enumerate(self.ranges):
            if range_val <= 0.0 or range_val > 12.0:  # Skip invalid readings
                continue
                
            # Calculate angle for this reading
            angle = self.angle_min + i * self.angle_increment + robot_yaw
            
            # Calculate endpoint of ray
            end_x = robot_x + range_val * math.cos(angle)
            end_y = robot_y + range_val * math.sin(angle)
            
            # Convert to map coordinates
            map_x = int((end_x - self.map_origin_x) / self.map_resolution)
            map_y = int((end_y - self.map_origin_y) / self.map_resolution)
            
            # Update map if within bounds
            if 0 <= map_x < self.map_size and 0 <= map_y < self.map_size:
                # Mark a small area around the detected point as occupied
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        nx, ny = map_x + dx, map_y + dy
                        if 0 <= nx < self.map_size and 0 <= ny < self.map_size:
                            self.map[ny, nx] = 100  # Mark as occupied
                
                # Mark cells along the ray as free
                self.mark_ray_free(robot_x, robot_y, end_x, end_y)
        
        # Publish updated map
        self.publish_map()

    def mark_ray_free(self, start_x, start_y, end_x, end_y):
        """Mark cells along a ray as free space"""
        # Convert to map coordinates
        start_map_x = int((start_x - self.map_origin_x) / self.map_resolution)
        start_map_y = int((start_y - self.map_origin_y) / self.map_resolution)
        end_map_x = int((end_x - self.map_origin_x) / self.map_resolution)
        end_map_y = int((end_y - self.map_origin_y) / self.map_resolution)
        
        # Use Bresenham's line algorithm to mark cells as free
        dx = abs(end_map_x - start_map_x)
        dy = abs(end_map_y - start_map_y)
        sx = 1 if start_map_x < end_map_x else -1
        sy = 1 if start_map_y < end_map_y else -1
        err = dx - dy
        
        x, y = start_map_x, start_map_y
        while x != end_map_x or y != end_map_y:
            if 0 <= x < self.map_size and 0 <= y < self.map_size:
                self.map[y, x] = 0  # Mark as free
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def publish_map(self):
        """Publish the current map as an OccupancyGrid message"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'odom'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_size
        map_msg.info.height = self.map_size
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Flatten and convert map to list
        map_msg.data = self.map.flatten().tolist()
        self.map_pub.publish(map_msg)

    def odom_cb(self, msg):
        """Update current pose and calculate turn angle"""
        prev_pose = self.current_pose
        self.current_pose = msg.pose.pose
        
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
                self.turn_direction *= -1
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
            # Get current orientation
            current_quat = [
                self.current_pose.orientation.w,
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z
            ]
            start_quat = [
                self.start_turn_pose.orientation.w,
                self.start_turn_pose.orientation.x,
                self.start_turn_pose.orientation.y,
                self.start_turn_pose.orientation.z
            ]
            
            _, _, current_yaw = transforms3d.euler.quat2euler(current_quat)
            _, _, start_yaw = transforms3d.euler.quat2euler(start_quat)
            
            # Calculate angle turned
            angle_turned = abs(self.wrap_to_pi(current_yaw - start_yaw))
            
            # Check if we've turned enough
            if angle_turned >= self.turn_angle - self.angle_threshold:
                self.get_logger().info('Turn completed')
                self.state = "NAVIGATING"
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                # Continue turning
                cmd.linear.x = 0.0
                cmd.angular.z = self.max_angular_vel * self.turn_direction
        
        # Publish command
        self.pub.publish(cmd)

    def wrap_to_pi(self, angle):
        """Wrap angle to [-π, π]"""
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = SLAMNavigator()
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
