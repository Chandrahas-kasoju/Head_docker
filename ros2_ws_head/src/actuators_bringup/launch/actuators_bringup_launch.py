import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('actuators_bringup'),
        'config',
        'actuators_bringup_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='actuators_bringup',
            executable='actuators',
            name='actuators',
            output='screen',
            parameters=[config],
        ),
    ])
