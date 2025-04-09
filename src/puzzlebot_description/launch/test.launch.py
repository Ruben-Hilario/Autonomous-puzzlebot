import os
import yaml

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'],description='Use sim time'),
    DeclareLaunchArgument('world', default_value='simple_world', description='Ignition World'),
    DeclareLaunchArgument('robot_name', default_value='puzzlebot', description='Ignition model name'),
]

def generate_launch_description():
    # Directories
    pkg_gazebo = get_package_share_directory('puzzlebot_description')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_gz_sim')
    gazebo_path = "/home/testeo/src/puzzlebot_description/gazebo/"
    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_gazebo, 'gazebo') + ':' + gazebo_path + ':' +'$GZ_SIM_RESOURCE_PATH'])
    
    ign_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            os.path.join(pkg_gazebo, 'gazebo/plugins'), ':' + '$GZ_SIM_SYSTEM_PLUGIN_PATH'])

    # Path to ignition launch
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'gz_sim.launch.py'])

    # Launch Ignition Gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),'.sdf',' -r',' -v 4'])
        ]
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
        ]
    )

    # Launch the robot model and bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_robot_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/VelocityEncR@std_msgs/msg/Float32[gz.msgs.Float',
            '/VelocityEncL@std_msgs/msg/Float32[gz.msgs.Float'
        ],
        output='screen'
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)
    ld.add_action(ignition_gazebo)
    ld.add_action(clock_bridge)
    ld.add_action(bridge)
    
    return ld
