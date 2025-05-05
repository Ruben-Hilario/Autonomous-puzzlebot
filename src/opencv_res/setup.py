from setuptools import find_packages, setup

package_name = 'opencv_res'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brad',
    maintainer_email='hilarioruben09@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_tracker = opencv_res.ball_tracker:main',
            'ball_decision = opencv_res.ball_decision:main',
        ],
    },
)
