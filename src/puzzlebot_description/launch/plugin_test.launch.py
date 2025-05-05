from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_file_name = 'plugin_test.sdf'
    world_path = os.path.join(
        get_package_share_directory('puzzlebot_description'), 
        'models',
        world_file_name
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_path],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])
