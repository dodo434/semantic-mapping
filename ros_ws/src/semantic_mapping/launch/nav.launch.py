from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('semantic_mapping'))
    defaults = {
        'params_file': str(share / 'config' / 'nav2.yaml'), 'use_sim_time': 'false',
        'autostart': 'false', 'enable_velocity_gate': 'false',
        'base_command': '/semantic/bench_velocity',
    }
    declared = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    params = LaunchConfiguration('params_file')
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    lifecycle = ['planner_server', 'controller_server', 'bt_navigator']
    nodes = [
        Node(package='nav2_planner', executable='planner_server', name=lifecycle[0],
             output='screen', parameters=[params, clock]),
        Node(package='nav2_controller', executable='controller_server', name=lifecycle[1],
             output='screen', parameters=[params, clock],
             remappings=[('cmd_vel', '/semantic/nav_velocity')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name=lifecycle[2],
             output='screen', parameters=[params, clock, {
                 'default_nav_to_pose_bt_xml': str(share / 'config' / 'navigate.xml'),
                 'default_nav_through_poses_bt_xml': str(share / 'config' / 'navigate-through.xml'),
             }]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', parameters=[clock, {
                 'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                 'node_names': lifecycle,
             }]),
        Node(package='semantic_mapping', executable='velocity_gate', name='velocity_gate',
             output='screen', parameters=[clock, {
                 'enabled': ParameterValue(LaunchConfiguration('enable_velocity_gate'), value_type=bool),
             }], remappings=[('base_command', LaunchConfiguration('base_command'))]),
    ]
    return LaunchDescription(declared + nodes)
