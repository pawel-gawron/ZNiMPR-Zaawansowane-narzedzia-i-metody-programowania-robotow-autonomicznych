import launch
import launch_ros.actions
from launch.actions import ExecuteProcess

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    noise_filter_launch_pkg_prefix = get_package_share_directory("noise_filter")

    noise_filter_config_param = DeclareLaunchArgument(
        'noise_filter_config_param_file',
        default_value=[noise_filter_launch_pkg_prefix, '/config/defaults.param.yaml'],
        description='Node config.'
    )
            
    return launch.LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', 'rosbag/rosbag2_2022_11_21-11_38_22_0.db3', '--loop'],
            output='screen',
        ),
        noise_filter_config_param,
        launch_ros.actions.Node(
            package='noise_filter',
            executable='noise_filter_node_exe',
            name='noise_filter_launch',
            output='screen',
            remappings=[
                ("input_cloud", "/phoxi/phoxi_camera/cloud"),
                ("output_cloud", "~/cloud_filtered")
                ],
            parameters=[
                LaunchConfiguration('noise_filter_config_param_file')
                ],
            arguments=['--log-level', 'info', '--enable-stdout-logs']),
        launch_ros.actions.Node(
            package='gawron_plane_seg',
            executable='gawron_plane_seg_node_exe',
            name='gawron_plane_seg_launch',
            arguments=['--log-level', 'info', '--enable-stdout-logs'],
            output='screen'),
        launch_ros.actions.Node(
            package='gawron_plane_meta',
            executable='gawron_plane_meta_node_exe',
            name='gawron_plane_meta_launch',
            arguments=['--log-level', 'info', '--enable-stdout-logs'],
            output='screen'),
  ])