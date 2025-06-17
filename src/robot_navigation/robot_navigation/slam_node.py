#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from tf2_ros import TransformBroadcaster
import tf2_ros
import tf2_geometry_msgs
from rclpy import qos

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        # Parameters
        self.map_resolution = 0.05  # meters per pixel
        self.map_width = 1000  # pixels
        self.map_height = 1000  # pixels
        self.map_origin_x = -25.0  # meters
        self.map_origin_y = -25.0  # meters
        
        # Initialize map
        self.occupancy_grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.occupancy_grid.fill(-1)  # -1 means unknown
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos.qos_profile_sensor_data
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos.qos_profile_sensor_data
        )
        
        # Publishers
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            'map',
            qos.qos_profile_sensor_data
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Robot pose
        self.robot_pose = None
        
        # Timer for map publishing
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('SLAM node initialized')
    
    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.robot_pose = msg.pose.pose
        
        # Broadcast transform from odom to base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
    
    def scan_callback(self, msg):
        """Process LiDAR scan and update occupancy grid"""
        if self.robot_pose is None:
            return
            
        # Get robot position in map coordinates
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        
        # Convert robot position to grid coordinates
        grid_x = int((robot_x - self.map_origin_x) / self.map_resolution)
        grid_y = int((robot_y - self.map_origin_y) / self.map_resolution)
        
        # Process each laser reading
        for i, range_reading in enumerate(msg.ranges):
            if range_reading < msg.range_min or range_reading > msg.range_max:
                continue
                
            # Calculate angle for this reading
            angle = msg.angle_min + i * msg.angle_increment
            
            # Calculate endpoint of laser beam in world coordinates
            end_x = robot_x + range_reading * np.cos(angle)
            end_y = robot_y + range_reading * np.sin(angle)
            
            # Convert endpoint to grid coordinates
            end_grid_x = int((end_x - self.map_origin_x) / self.map_resolution)
            end_grid_y = int((end_y - self.map_origin_y) / self.map_resolution)
            
            # Update occupancy grid using Bresenham's line algorithm
            self.update_grid_line(grid_x, grid_y, end_grid_x, end_grid_y, range_reading)
    
    def update_grid_line(self, x0, y0, x1, y1, range_reading):
        """Update occupancy grid using Bresenham's line algorithm"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        dx *= 2
        dy *= 2
        
        # Update cells along the line
        for _ in range(n):
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                # If this is the endpoint (obstacle)
                if x == x1 and y == y1:
                    self.occupancy_grid[y, x] = 100  # Occupied
                # If this is along the line (free space)
                else:
                    if self.occupancy_grid[y, x] == -1:  # Only update unknown cells
                        self.occupancy_grid[y, x] = 0  # Free
            
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
    
    def publish_map(self):
        """Publish the occupancy grid map"""
        if self.robot_pose is None:
            return
            
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Convert numpy array to list
        map_msg.data = self.occupancy_grid.flatten().tolist()
        
        self.map_pub.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()