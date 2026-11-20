from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'camera': '/camera/camera', 'model': '', 'device': '0', 'image_size': '416',
        'max_hz': '15.0', 'map_hz': '8.0', 'pixel_stride': '8', 'base_height': '0.18',
        'enable_fusion': 'true', 'publish_filtered_depth': 'false', 'publish_static_cloud': 'false', 'use_sim_time': 'false',
    }
    arguments = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    camera = LaunchConfiguration('camera')
    def typed(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    common = {'use_sim_time': typed('use_sim_time', bool)}
    return LaunchDescription(arguments + [
        Node(
            package='semantic_mapping', executable='segmentation', namespace='semantic',
            output='screen', parameters=[common, {
                'model': typed('model', str), 'device': typed('device', str),
                'image_size': typed('image_size', int), 'max_hz': typed('max_hz', float),
            }], remappings=[('color', [camera, '/color/image_raw'])],
        ),
        Node(
            package='semantic_mapping', executable='semantic_fusion', namespace='semantic',
            condition=IfCondition(LaunchConfiguration('enable_fusion')),
            output='screen', parameters=[common, {
                'base_height': typed('base_height', float), 'map_hz': typed('map_hz', float),
                'pixel_stride': typed('pixel_stride', int),
                'publish_filtered_depth': typed('publish_filtered_depth', bool),
                'publish_static_cloud': typed('publish_static_cloud', bool),
            }], remappings=[
                ('aligned_depth', [camera, '/aligned_depth_to_color/image_raw']),
                ('depth_info', [camera, '/aligned_depth_to_color/camera_info']),
                ('left_info', [camera, '/infra1/camera_info']),
                ('right_info', [camera, '/infra2/camera_info']),
            ],
        ),
    ])
