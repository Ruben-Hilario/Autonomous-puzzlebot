#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import localization
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from tf_transformations import quaternion_from_euler

class Joint_State_tf(Node):
    def __init__(self):
        super().__init__('Joint_State_tf')

        # Publisher 
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Transform broadcaster
        self.br = TransformBroadcaster(self)

        # Wheel parameters
        self.wheel_base = 0.18
        self.wheel_radius = 0.15

        # Initial wheel rotation (angle in radians)
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        # Velocity values for the wheels (rad/s)
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        # Define the positions of the wheels relative to the robot base (base_link)
        self.left_wheel_offset = self.wheel_base / 2.0  # Left wheel offset along x-axis
        self.right_wheel_offset = -self.wheel_base / 2.0 # Right wheel offset in the opposite direction

        # Subscriber
        self.odom_sub = self.create_subscription(localization, 'odom', self.odometry_callback, 1)
        self.left_wheel_sub = self.create_subscription(Float32, 'VelocityEncL', self.left_wheel_callback, 1)
        self.right_wheel_sub = self.create_subscription(Float32, 'VelocityEncR', self.right_wheel_callback, 1)

        # # Start the timer now
        # self.start_time = self.get_clock().now()
        # time_period = 0.1
        # self.timer = self.create_timer(time_period, self.odometry_callback)

    def odometry_callback(self, msg):

        # Create a JointState message
        self.joint_state = JointState()
        self.joint_state.header = msg.header  # Use the same timestamp and frame
        self.joint_state.name = ['wheel_left_joint', 'wheel_right_joint']
        # These values would ideally be calculated based on the robot's specific kinematics
        self.joint_state.position = [0., 0.]  # Placeholder values
        self.joint_state.velocity = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        self.joint_state.effort = []

        # Publish joint states
        self.joint_pub.publish(self.joint_state)

        current_time = self.get_clock().now()

        # Publish transform using tf2 (from 'odom' to 'base_link')
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        
        self.br.sendTransform(t)

        self.left_wheel_angle += self.left_wheel_velocity * 0.1 
        self.left_wheel_angle = self.left_wheel_angle % (2 * math.pi)
        left_wheel_rotation = quaternion_from_euler(0.0, self.left_wheel_angle, 0.0)

        lt = TransformStamped()
        lt.header.stamp = current_time.to_msg()
        lt.header.frame_id = 'base_link'
        lt.child_frame_id = 'left_wheel'
        lt.transform.translation.x = 0.05
        lt.transform.translation.y = self.left_wheel_offset
        lt.transform.translation.z = 0.05
        lt.transform.rotation.x = left_wheel_rotation[0]
        lt.transform.rotation.y = left_wheel_rotation[1]
        lt.transform.rotation.z = left_wheel_rotation[2]
        lt.transform.rotation.w = left_wheel_rotation[3]

        self.br.sendTransform(lt)

        self.right_wheel_angle += self.right_wheel_velocity * 0.1 
        self.right_wheel_angle = self.right_wheel_angle % (2 * math.pi)
        right_wheel_rotation = quaternion_from_euler(0.0, self.right_wheel_angle, 0.0)

        rt = TransformStamped()
        rt.header.stamp = current_time.to_msg()
        rt.header.frame_id = 'base_link'
        rt.child_frame_id = 'right_wheel'
        rt.transform.translation.x = 0.05
        rt.transform.translation.y = self.right_wheel_offset
        rt.transform.translation.z = 0.05
        rt.transform.rotation.x = right_wheel_rotation[0]
        rt.transform.rotation.y = right_wheel_rotation[1]
        rt.transform.rotation.z = right_wheel_rotation[2]
        rt.transform.rotation.w = right_wheel_rotation[3]

        self.br.sendTransform(rt)

    def left_wheel_callback(self, msg):
        self.left_wheel_velocity = msg.data

    def right_wheel_callback(self, msg):
        self.right_wheel_velocity = msg.data

        
    
def main(args= None):
    rclpy.init(args=args)
    joint_tf = Joint_State_tf()
    rclpy.spin(joint_tf)
    joint_tf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

   