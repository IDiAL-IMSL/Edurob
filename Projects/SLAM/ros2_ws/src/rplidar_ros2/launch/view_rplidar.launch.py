from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rplidar.launch.py'])),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', [ThisLaunchFileDir(), '/../rviz/rplidar.rviz']],
        )
    ])
