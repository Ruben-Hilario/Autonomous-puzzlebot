# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')

        self.wait_for_ros_time()

        # Publisher to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        #Declare parameters
        self.declare_parameter('linear_speed',0.2)
        self.declare_parameter('angular_speed',0.5)
        self.declare_parameter('waypoints',[[1.0,1.0],[0.0,1.0],[0.0,0.0]])
        self.add_on_set_parameters_callback(self.parameters_callback)
        # Time-based control variables
        self.state = 0  # 0: rotate, 1: forward, 2: stop
        self.state_start_time = self.get_clock().now()

        # Current position and waypoints
        self.x = 0.0
        self.y = 0.0
        self.waypoints = self.get_parameter('waypoints').value  # Updated waypoints
        self.current_waypoint_index = 0
        
        # Define speeds
        self.linear_speed = self.get_parameter('linear_speed').value  # m/s
        self.angular_speed = self.get_parameter('angular_speed').value  # rad/s
        
        # Initialize movement parameters for first waypoint
        self.update_movement_parameters()
        
        # Timer to update state machine
        self.timer_period = 0.2  # 10 Hz control loop
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info('Open loop controller initialized!')
        
    def update_movement_parameters(self):
        """Calculate movement parameters for current waypoint"""
        if self.current_waypoint_index < len(self.waypoints):
            self.final_x, self.final_y = self.waypoints[self.current_waypoint_index]
            self.angle = np.arctan2((self.final_y-self.y), (self.final_x-self.x))
            self.distance = np.sqrt((self.final_x-self.x)**2 + (self.final_y-self.y)**2)
            self.forward_time = self.distance / self.linear_speed
            if self.x ==0.0:
                self.rotate_time = abs(self.angle/30) / self.angular_speed
            else:
                self.rotate_time = abs(self.angle) / self.angular_speed
            self.get_logger().info(f'Moving to waypoint {self.current_waypoint_index+1}: ({self.final_x}, {self.final_y})')
    
    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9
        cmd = Twist()
        
        if self.state == 0:
            # Rotate to face the goal
            cmd.angular.z = np.sign(self.angle) * self.angular_speed
            self.get_logger().info('Rotating to face goal...')
            if elapsed_time >= self.rotate_time:
                self.state = 1
                self.state_start_time = now
                self.get_logger().info('Finished rotation. Moving forward...')
                
        elif self.state == 1:
            # Move forward to goal
            cmd.linear.x = self.linear_speed
            self.get_logger().info('Moving forward to goal...')
            if elapsed_time >= self.forward_time:
                # Update position to current waypoint
                self.x = self.final_x
                self.y = self.final_y
                
                # Move to next waypoint or stop
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints):
                    self.update_movement_parameters()
                    self.state = 0  # Start with rotation for next waypoint
                    self.state_start_time = now
                    self.get_logger().info('Starting movement to next waypoint...')
                else:
                    self.state = 2  # All waypoints reached
                    self.state_start_time = now
                    self.get_logger().info('All waypoints reached. Stopping...')
                    
        elif self.state == 2:
            # Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Stopped.')
            self.timer.cancel()  # Stop the control loop

        self.cmd_vel_pub.publish(cmd)

    def wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi), (2 * np.pi))
        if (result < 0):
            result += 2 * np.pi
        return result - np.pi

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'ROS time is active! Start time: {now.nanoseconds * 1e-9:.2f}s')

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'linear_speed':
                self.linear_speed = param.value
                self.get_logger().info(f"Updated linear_speed to {self.linear_speed}")
            elif param.name == 'angular_speed':
                self.angular_speed = param.value
                self.get_logger().info(f"Updated angular_speed to {self.angular_speed}")
            elif param.name == 'waypoints':
                try:
                    # Verificar que todos los waypoints tengan exactamente 2 coordenadas
                    if all(len(point) == 2 for point in param.value):
                        self.waypoints = param.value
                        self.get_logger().info(f"Updated waypoints to {self.waypoints}")
                        self.current_waypoint_index = 0
                        self.update_movement_parameters()
                    else:
                        self.get_logger().error("All waypoints must have exactly 2 coordinates (x, y)")
                except Exception as e:
                    self.get_logger().error(f"Invalid waypoints format: {str(e)}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCtrl()
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
