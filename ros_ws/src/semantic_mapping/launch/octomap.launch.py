from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='octomap_server', executable='octomap_server_node', name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'map', 'resolution': 0.05, 'sensor_model.max_range': 5.0,
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            remappings=[('cloud_in', '/semantic/static_cloud')],
        ),
    ])
