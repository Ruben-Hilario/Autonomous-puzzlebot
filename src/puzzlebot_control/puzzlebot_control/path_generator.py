import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        # Publisher to send calculated angles and distances
        self.path_pub = self.create_publisher(Twist, 'pose', 10)

        # Declare parameters
        self.declare_parameter('waypoints', [1.0, 1.0, 0.0, 1.0, 1.5, 1.5, 2.0, 2.0, 2.0, 1.0])

        # Current position and waypoints
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        flat = self.get_parameter('waypoints').value
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        self.current_waypoint_index = 0

        self.timer_period = 1.0  # 1 Hz
        self.timer = self.create_timer(self.timer_period, self.generate_path)

    def generate_path(self):
        if self.current_waypoint_index < len(self.waypoints):
            final_x, final_y = self.waypoints[self.current_waypoint_index]
            target_angle = np.arctan2((final_y - self.y), (final_x - self.x))
            angle_to_rotate = self.wrap_to_Pi(target_angle - self.theta)
            distance_to_travel = np.sqrt((final_x - self.x)**2 + (final_y - self.y)**2)
            
            # Publish rotation angle and distance to travel
            path_msg = Twist()
            path_msg.linear.x = distance_to_travel
            path_msg.angular.z = angle_to_rotate
            self.path_pub.publish(path_msg)

            self.get_logger().info(f"Published path: distance {distance_to_travel}, rotation angle {angle_to_rotate}")

            self.current_waypoint_index += 1  # Move to the next waypoint
        else:
            self.get_logger().info("All waypoints processed.")

    def wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi), (2 * np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
