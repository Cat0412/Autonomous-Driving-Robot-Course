from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('fallen_person_safety'))
    workspace_root = package_share.parents[3]
    default_model = str(workspace_root / 'best.pt')
    config_file = str(package_share / 'config' / 'safety.yaml')

    model_path = LaunchConfiguration('model_path')
    start_admin_monitor = LaunchConfiguration('start_admin_monitor')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model,
            description='Absolute path to the trained YOLO best.pt model',
        ),
        DeclareLaunchArgument(
            'start_admin_monitor',
            default_value='true',
            description='Start the local administrator event/snapshot receiver',
        ),
        Node(
            package='fallen_person_safety',
            executable='safety_controller',
            name='fallen_person_safety',
            output='screen',
            parameters=[
                config_file,
                {'model_path': model_path},
            ],
        ),
        Node(
            package='fallen_person_safety',
            executable='admin_monitor',
            name='fallen_person_admin_monitor',
            output='screen',
            condition=IfCondition(start_admin_monitor),
        ),
    ])
