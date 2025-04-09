from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('puzzlebot_description'),
            'launch',
            'gazebo.launch.py'
        )
    )

    # Open loop controller node
    open_loop_controller = Node(
        package='puzzlebot_control',
        executable='open_loop_control',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        open_loop_controller
    ])