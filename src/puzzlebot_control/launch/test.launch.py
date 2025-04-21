from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    odom = Node(
        package='puzzlebot_localisation',
        executable='puzzlebot_odometry',
        name='puzzlebot_odometry',    
    )
    controller_node = Node(
        package='puzzlebot_control',
        executable='open_loop_control',
        name='open_loop_control',
    )
    return LaunchDescription([odom,
        controller_node,]
    )