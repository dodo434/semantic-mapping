from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('semantic_mapping'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        ComposableNodeContainer(
            name='semantic_nvblox_container', namespace='', package='rclcpp_components',
            executable='component_container_mt', output='screen',
            composable_node_descriptions=[ComposableNode(
                package='nvblox_ros', plugin='nvblox::NvbloxNode', name='nvblox_node',
                parameters=[str(share / 'config' / 'nvblox.yaml'), {
                    'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                }], remappings=[
                    ('camera_0/depth/image', '/semantic/static_depth'),
                    ('camera_0/depth/camera_info', '/semantic/static_depth_info'),
                ],
            )],
        ),
    ])
