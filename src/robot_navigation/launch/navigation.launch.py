#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # SLAM Node
        Node(
            package='robot_navigation',
            executable='slam_node',
            name='slam_node',
            output='screen'
        ),
        
        # Navigation Node
        Node(
            package='robot_navigation',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        ),
        
        # TF Static Publisher for map to odom transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    ])