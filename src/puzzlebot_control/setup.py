from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'conf'), glob(os.path.join('conf','*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mario Martinez',
    maintainer_email='mario.mtz@manchester-robotics.com',
    description='Puzzlebot Control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'open_loop_control = puzzlebot_control.open_loop_control:main',
            'route = puzzlebot_control.route:main',
            'path_generator = puzzlebot_control.path_generator:main',
            'test = puzzlebot_control.test:main',
            'ball_movement = puzzlebot_control.ball_test:main',
            'PID = puzzlebot_control.PID_Ball_Tracker:main',
            'ball_tracker = opencv_res.ball_tracker:main',
            'puzzlebot_odometry = puzzlebot_localisation.puzzlebot_odometry:main',
            'closed_loop_control = puzzlebot_control.closed_loop_control:main',
            'ball_route = puzzlebot_control.ball_route:main', #Cambiar el algoritmo
            'ball_decision = opencv_res.ball_decision:main', 
            'lidar = puzzlebot_control.lidar:main',
            'tl = puzzlebot_control.traffic_light:main',
            'line_follower = puzzlebot_control.line_follower:main'
        ],
    },
)
