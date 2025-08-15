import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sensors_bringup'),
        'config',
        'sensors_bringup_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='sensors_bringup',
            executable='person_detect',
            name='person_detect',
            output='screen',
            parameters=[config],
        ),
    ])
