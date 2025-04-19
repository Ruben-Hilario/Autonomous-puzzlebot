from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    odom = Node(
        package='puzzlebot_localisation',
        executable='puzzlebot_odometry',
        name='puzzlebot_odometry',
        output='screen',        
    )
    controller_node = Node(
        package='puzzlebot_control',
        executable='closed_loop_control',
        name='closed_loop_control',
        output='screen'
    )
    return LaunchDescription(
        odom,
        controller_node,
    )