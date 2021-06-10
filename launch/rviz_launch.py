# Copyright 2021 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
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
            namespace=LaunchConfiguration('namespace'),
            on_exit=Shutdown()
        ),
        Node(
            package='naosoccer_visualization',
            executable='ball_to_marker',
            namespace=LaunchConfiguration('namespace')
        ),
        Node(
            package='naosoccer_visualization',
            executable='goalpost_array_to_marker_array',
            namespace=LaunchConfiguration('namespace')
        ),
        Node(
            package='naosoccer_visualization',
            executable='field_line_array_to_marker_array',
            namespace=LaunchConfiguration('namespace')
        ),
        Node(
            package='naosoccer_visualization',
            executable='robot_array_to_marker_array',
            namespace=LaunchConfiguration('namespace')
        ),
        Node(
            package='naosoccer_visualization',
            executable='flag_array_to_marker_array',
            namespace=LaunchConfiguration('namespace')
        ),        
    ])
