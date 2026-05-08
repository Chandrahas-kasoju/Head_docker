from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensors_bringup',
            executable='camera_bringup',
            name='camera_node'
        ),
        Node(
            package='sensors_bringup',
            executable='thermal_viz',
            name='thermal_viz_node'
        ),
        #Added face tracker so we can test full pipeline
        Node(
            package='face_tracker',
            executable='tracker',
            name='face_tracker_node'
        )
    ])
