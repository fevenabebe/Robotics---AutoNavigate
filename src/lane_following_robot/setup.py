from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lane_following_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 package index resource
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[py]*'))),

        # Install world files
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.world'))),

        # Install robot model files
        (os.path.join('share', package_name, 'models', 'lane_bot'),
            glob(os.path.join('models', 'lane_bot', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='feven',
    maintainer_email='feven@todo.todo',
    description='Lane-following robot with camera, vision, and control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_driver = lane_following_robot.camera_driver:main',
            'image_processor = lane_following_robot.image_processor:main',
            'controller = lane_following_robot.controller:main',
            'visualizer = lane_following_robot.visualizer:main',
            'spawn_robot = lane_following_robot.spawn_robot:main',
        ],
    },
)
