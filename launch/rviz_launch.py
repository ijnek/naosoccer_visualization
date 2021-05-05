import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                os.path.join(
                    get_package_share_directory('naosoccer_visualization'),
                    'rviz', 'nao.rviz')
            ],
            namespace=LaunchConfiguration('namespace')
        ),
        Node(
            package='naosoccer_visualization',
            executable='ball_to_marker',
            namespace=LaunchConfiguration('namespace')
        ),
    ])
