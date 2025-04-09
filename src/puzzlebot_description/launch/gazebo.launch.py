import yaml
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions.path_join_substitution import PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='puzzlebot', description='Ignition model name'),
    DeclareLaunchArgument('world', default_value='world1', description='World name'),
]


def generate_launch_description():
    #Ignition
    pkg_gazebo = get_package_share_directory('puzzlebot_description')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_gz_sim')
    gazebo_path = "/home/testeo/src/puzzlebot_description/models/"
    #Ignition Resources
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_gazebo, 'models') + ':' + gazebo_path + ':' +'$GZ_SIM_RESOURCE_PATH']
    )
    
    ign_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            os.path.join(pkg_gazebo, 'models/plugins'), ':' + '$GZ_SIM_SYSTEM_PLUGIN_PATH'
        ]
    )
    ign_gazebo_launch = PathJoinSubstitution([pkg_ros_ign_gazebo, 'launch', 'gz_sim.launch.py'])
    

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_robot_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/VelocityEncR@std_msgs/msg/Float32[gz.msgs.Float',
                   '/VelocityEncL@std_msgs/msg/Float32[gz.msgs.Float'
                  ],
        output='screen'
    )
    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge', output='screen',
                        arguments=[
                        '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
    ])
    # Ignition gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),'.sdf',' -r',' -v 4'])
        ]
    )
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)
    ld.add_action(ignition_gazebo)
    ld.add_action(clock_bridge)
    ld.add_action(bridge)
    return ld