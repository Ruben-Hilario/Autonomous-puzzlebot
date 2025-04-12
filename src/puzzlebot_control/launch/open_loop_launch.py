from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path Generator node
    path_generator_node = Node(
        package='puzzlebot_control',
        executable='path_generator',  # Nombre del ejecutable para el nodo path_generator
        output='screen'
    )

    # Controller node
    controller_node = Node(
        package='puzzlebot_control',
        executable='test',  # Nombre del ejecutable para el nodo controller
        output='screen'
    )

    return LaunchDescription([
        path_generator_node,
        controller_node
    ])
