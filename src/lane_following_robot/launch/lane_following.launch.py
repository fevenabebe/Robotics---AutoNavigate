from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_lane_robot = get_package_share_directory('lane_following_robot')
    

    world_path = os.path.join(pkg_lane_robot, 'worlds', 'lane_world.world')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    return LaunchDescription([
        gazebo_launch,
        ExecuteProcess(
            cmd=['ros2', 'run', 'lane_following_robot', 'spawn_robot'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'lane_following_robot', 'controller'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'lane_following_robot', 'image_processor'],
            output='screen'
        )
    ])
