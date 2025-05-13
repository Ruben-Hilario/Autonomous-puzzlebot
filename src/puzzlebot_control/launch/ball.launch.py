from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ball_tracker_node = Node(
        package='opencv_res',
        executable='ball_tracker',
        output = 'screen',
    )
    ball_movement = Node(
        package='puzzlebot_control',
        executable='ball_movement'
    )
    odom = Node(
        package='puzzlebot_localisation',
        executable='puzzlebot_odometry',
        name='puzzlebot_odometry',    
    )
    controller_node = Node(
        package='puzzlebot_control',
        executable='PID', 
        output='screen'
    )

    return LaunchDescription([
        odom,
        ball_tracker_node,
        ball_movement,
        controller_node,
    ])
