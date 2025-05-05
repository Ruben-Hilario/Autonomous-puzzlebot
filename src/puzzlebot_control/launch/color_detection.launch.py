from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    color_detection_node = Node(
        package='opencv_res',
        executable='ball_decision',
        output='screen',
    )
    odom_node = Node(
        package='puzzlebot_localisation',
        executable='puzzlebot_odometry',
        output='screen',
    )
    controller_node = Node(
        package='puzzlebot_control',
        executable='ball_route', 
        output='screen'
    )

    return LaunchDescription([
        odom_node,
        color_detection_node,
        controller_node,
    ])
