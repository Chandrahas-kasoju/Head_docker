import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('face_tracker'),
        'config',
        'face_tracker_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='face_tracker',
            executable='face_tracker',
            name='face_tracker',
            output='screen',
            parameters=[config],
        ),
    ])
