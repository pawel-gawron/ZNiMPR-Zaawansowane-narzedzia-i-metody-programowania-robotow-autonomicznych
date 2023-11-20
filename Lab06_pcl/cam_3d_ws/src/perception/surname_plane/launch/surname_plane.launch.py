# # Copyright 2023 Pawel_Gawron
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.actions import OpaqueFunction
# from launch.substitutions import LaunchConfiguration
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def launch_setup(context, *args, **kwargs):
#     param_path = LaunchConfiguration('surname_plane_param_file').perform(context)
#     if not param_path:
#         param_path = PathJoinSubstitution(
#             [FindPackageShare('surname_plane'), 'config', 'surname_plane.param.yaml']
#         ).perform(context)

#     surname_plane_node = Node(
#         package='surname_plane',
#         executable='surname_plane_node_exe',
#         name='surname_plane_node',
#         parameters=[
#             param_path
#         ],
#         output='screen',
#         arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
#     )

#     return [
#         surname_plane_node
#     ]


# def generate_launch_description():
#     declared_arguments = []

#     def add_launch_arg(name: str, default_value: str = None):
#         declared_arguments.append(
#             DeclareLaunchArgument(name, default_value=default_value)
#         )

#     add_launch_arg('surname_plane_param_file', '')

#     return LaunchDescription([
#         *declared_arguments,
#         OpaqueFunction(function=launch_setup)
#     ])


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
    surname_plane_launch_pkg_prefix = get_package_share_directory("surname_plane")

    surname_plane_config_param = DeclareLaunchArgument(
        'surname_plane_config_param_file',
        default_value=[surname_plane_launch_pkg_prefix, '/config/surname_plane.param.yaml'],
        description='Node config.'
    )

    surname_plane_node = Node(
        package='surname_plane',
        executable='surname_plane_node_exe',
        name='surname_plane',
        output='screen',
        remappings=[
                ("input_cloud", "/phoxi/phoxi_camera/cloud"),
                ("output_cloud", "~/cloud_filtered")
                ],
        parameters=[
            LaunchConfiguration('surname_plane_config_param_file')
        ],
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs']
    )

    ld = LaunchDescription([
        surname_plane_config_param,
        surname_plane_node,
    ])
    return ld
