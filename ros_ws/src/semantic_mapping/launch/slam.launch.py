from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'camera': '/camera/camera', 'trajectory': '', 'use_masks': 'false',
        'use_wheel_odom': 'true', 'publish_tf': 'false', 'use_sim_time': 'false',
    }
    arguments = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    arguments.extend(DeclareLaunchArgument(name) for name in ('vocabulary', 'settings'))
    camera = LaunchConfiguration('camera')
    booleans = ('use_masks', 'use_wheel_odom', 'publish_tf', 'use_sim_time')
    params = {name: ParameterValue(LaunchConfiguration(name), value_type=bool) for name in booleans}
    params.update({name: LaunchConfiguration(name) for name in ('vocabulary', 'settings', 'trajectory')})
    return LaunchDescription(arguments + [Node(
        package='semantic_mapping', executable='stereo_inertial', namespace='semantic',
        output='screen', parameters=[params], remappings=[
            ('left_image', [camera, '/infra1/image_rect_raw']),
            ('right_image', [camera, '/infra2/image_rect_raw']),
            ('imu', [camera, '/imu']),
        ],
    )])
