import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for starting all the required nodes.

    Returns:
        LaunchDescription: The launch description object.
    """

    # Get the path to the launch file of the ti_mmwave_rospkg package
    ti_mmwave_rospkg_launch_file = os.path.join(
        get_package_share_directory('ti_mmwave_rospkg'),
        'launch',
        '6843AOP_FineMotion.py'  # Assuming a standard launch file name and location
    )

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
        Node(
            package='sensors_bringup',
            executable='static_transform',
            name='static_transform_node'
        ),
        Node(
            package='head_bringup',
            executable='face',
            name='face_node'
        ),
        # Include the launch file for the radar node
        # IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(ti_mmwave_rospkg_launch_file)
        # ),
    ])