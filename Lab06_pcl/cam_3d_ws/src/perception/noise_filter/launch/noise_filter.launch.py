# Copyright 2022 Amadeusz Szymko
# Perception for Physical Interaction Laboratory at Poznan University of Technology
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    noise_filter_launch_pkg_prefix = get_package_share_directory("noise_filter")

    noise_filter_config_param = DeclareLaunchArgument(
        'noise_filter_config_param_file',
        default_value=[noise_filter_launch_pkg_prefix, '/config/defaults.param.yaml'],
        description='Node config.'
    )

    noise_filter_node = Node(
        package='noise_filter',
        executable='noise_filter_node_exe',
        name='noise_filter',
        output='screen',
        remappings=[
                ("input_cloud", "/phoxi/phoxi_camera/cloud"),
                ("output_cloud", "~/output_cloud_filtered"),
                ("output_cloud_downsampled", "~/output_cloud_downsampled")
                ],
        parameters=[
            LaunchConfiguration('noise_filter_config_param_file')
        ],
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs']
    )

    ld = LaunchDescription([
        noise_filter_config_param,
        noise_filter_node,
    ])
    return ld
