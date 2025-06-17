#Keep it simple launch file
import yaml
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.path_join_substitution import PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true'  , choices=['true', 'false'], description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='puzzlebot', description='Ignition model name'),
    DeclareLaunchArgument('world', default_value='ss_world', description='World name'),
]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    world = LaunchConfiguration('world')
    pkg_gazebo = get_package_share_directory('puzzlebot_description')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_gz_sim')
    gazebo_path = get_package_share_directory('puzzlebot_description') + '/models/' #"/home/testeo/src/puzzlebot_description/models/"

    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_gazebo, 'models') + ':' + gazebo_path + ':' + '$GZ_SIM_RESOURCE_PATH']
    )
    
    ign_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[os.path.join(pkg_gazebo, 'models/plugins') + ':' + '$GZ_SIM_SYSTEM_PLUGIN_PATH']
    )

    ign_gazebo_launch = PathJoinSubstitution([pkg_ros_ign_gazebo, 'launch', 'gz_sim.launch.py'])

    # Bridge for cmd_vel and encoders
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

    # Clock Bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    )

    # Launch Ignition Gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), '.sdf -r -v 4']
        }.items()
    )
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/', world,'/model/', robot_name,'/link/chassis/sensor/camera/image' +'@sensor_msgs/msg/Image' +'[ignition.msgs.Image'],
            ['/world/', world,'/model/', robot_name,'/link/chassis/sensor/camera/camera_info' +'@sensor_msgs/msg/CameraInfo' +'[ignition.msgs.CameraInfo'],
            ],
        remappings=[
            (['/world/', world,'/model/', robot_name,'/link/chassis/sensor/camera/image'],'/video_source/raw'),
            (['/world/', world,'/model/', robot_name,'/link/chassis/sensor/camera/camera_info'],'/camera_info')
            ]
    )
    
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/', world,'/model/', robot_name,'/link/chassis/sensor/rplidar/scan' +'@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
        ],
        remappings=[(
            ['/world/', world,'/model/', robot_name,'/link/chassis/sensor/rplidar/scan'],'scan'
        )]
    )
    
    return LaunchDescription([*ARGUMENTS,
        camera_bridge,
        lidar_bridge,
        ign_resource_path,
        ign_gui_plugin_path,
        ignition_gazebo,
        clock_bridge,
        bridge,
    ])
