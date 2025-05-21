from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    line_tracker = Node(
        package='opencv_res',
        executable='line_tracker',
        output = 'screen',
    )
    
    line_control = Node(
        package='puzzlebot_control',
        executable='line_follower', 
        output='screen'
    )

    return LaunchDescription([
        line_tracker,
        line_control
    ])
