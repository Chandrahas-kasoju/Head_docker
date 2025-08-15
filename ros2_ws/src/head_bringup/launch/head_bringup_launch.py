from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for starting all the required nodes.

    Returns:
        LaunchDescription: The launch description object.
    """
    return LaunchDescription([
        Node(
            package='sensors_bringup',
            executable='camera_bringup',
            name='camera_node'
        ),
        Node(
            package='face_tracker',
            executable='dl_tracker',
            name='face_tracker_node'
        ),
        Node(
            package='actuators_bringup',
            executable='actuators',
            name='actuators_node'
        ),
    ])