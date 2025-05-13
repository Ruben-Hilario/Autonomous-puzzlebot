from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ball_decision = Node(
        package='opencv_res',
        executable='ball_decision',
        output = 'screen',
    )
    
    controller_node = Node(
        package='puzzlebot_control',
        executable='ball_route', 
        output='screen'
    )
    prueba = Node(
        package='puzzlebot_control',
        executable='closed_loop_control',
        output='screen'
    )
    odometry_node = Node(
        package='puzzlebot_localisation',
        executable='puzzlebot_odometry',
        output='screen'
    )

    return LaunchDescription([
        odometry_node,
        ball_decision,
        #prueba,
        controller_node
    ])
