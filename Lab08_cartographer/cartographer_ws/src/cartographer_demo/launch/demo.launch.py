# Copyright 2023 Amadeusz Szymko
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    configuration_basename = None

    # if IfCondition(LaunchConfiguration('mapping')).evaluate(context):
    #     configuration_basename = 'mapping.lua'
    #     print('Mapping mode')
    #     print(configuration_basename)
    # else:
    #     configuration_basename = 'localization.lua'
    #     path_to_map = LaunchConfiguration('map_path').perform(context)
    #     print('Localization mode')
    #     print(path_to_map)

    pkg_prefix = FindPackageShare('cartographer_demo')
    rviz_config = PathJoinSubstitution(
        [pkg_prefix, 'rviz', LaunchConfiguration('rviz_config')])

    # Cartographer
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [pkg_prefix, 'launch', 'cartographer.launch.py']
            )
        ),
        launch_arguments={
            # add args to dictionary here
        }.items()
    )

    # Tools
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', str(rviz_config.perform(context))
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return [
        rviz2,
        cartographer_launch
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )
    # Tools
    add_launch_arg('rviz', 'true')
    add_launch_arg('rviz_config', 'cartographer.rviz')
    add_launch_arg('mapping', 'true')

    add_launch_arg('map_path', '')
    # Localization
    # TODO: add args

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
