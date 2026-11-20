from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    launch_dir = Path(get_package_share_directory('semantic_mapping')) / 'launch'
    defaults = {
        'camera': '/camera/camera', 'trajectory': '', 'device': '0', 'image_size': '416',
        'max_hz': '15.0', 'map_hz': '8.0', 'pixel_stride': '8', 'base_height': '0.18',
        'use_wheel_odom': 'true', 'use_sim_time': 'false', 'with_nav2': 'false',
    }
    declared = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    declared += [DeclareLaunchArgument(name) for name in ('vocabulary', 'settings', 'model')]
    declared.append(DeclareLaunchArgument('geometry', default_value='none',
                                         choices=['none', 'nvblox', 'octomap']))

    def launch(name, arguments, condition=None, **extra):
        variables = {key: LaunchConfiguration(key) for key in arguments}
        return GroupAction(condition=condition, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / f'{name}.launch.py')),
            launch_arguments=(variables | extra).items(),
        )])

    use_gpu = PythonExpression(["'", LaunchConfiguration('geometry'), "' == 'nvblox'"])
    use_cpu = PythonExpression(["'", LaunchConfiguration('geometry'), "' == 'octomap'"])
    return LaunchDescription(declared + [
        launch('slam', ('camera', 'vocabulary', 'settings', 'trajectory', 'use_wheel_odom',
                        'use_sim_time'), publish_tf='true', use_masks='true'),
        launch('perception', ('camera', 'model', 'device', 'image_size', 'max_hz', 'map_hz',
                              'pixel_stride', 'base_height', 'use_sim_time'),
               publish_filtered_depth=use_gpu, publish_static_cloud=use_cpu),
        launch('nvblox', ('use_sim_time',), IfCondition(use_gpu)),
        launch('octomap', ('use_sim_time',), IfCondition(use_cpu)),
        launch('nav', ('use_sim_time',), IfCondition(LaunchConfiguration('with_nav2')),
               autostart='true'),
    ])
