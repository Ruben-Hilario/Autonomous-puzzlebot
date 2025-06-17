from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    odom = Node(
        package='puzzlebot_localisation',
        executable='puzzlebot_odometry',
        name='puzzlebot_odometry',    
    )
    lidar_node = Node(
        package='puzzlebot_control',
        executable='lidar',
        name='lidar'
    )
    return LaunchDescription([
        odom,
        lidar_node,
        
    ])