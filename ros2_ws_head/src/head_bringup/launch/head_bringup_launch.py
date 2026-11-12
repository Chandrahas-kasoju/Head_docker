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

    # Radar: radar_vitals publishes the point cloud and vital signs. It needs the
    # out-of-box mmw demo firmware on the IWR6843AOP (not the people tracking one).
    radar_vitals_launch_file = os.path.join(
       get_package_share_directory('radar_vitals'),
       'launch',
       'vitals.launch.py'
    )

    return LaunchDescription([
        Node(
            package='sensors_bringup',
            executable='camera_bringup',
            name='camera_node'
        ),
        Node(
            package='face_tracker',
            executable='face_tracker',
            name='face_tracker_node'
        ),
        Node(
            package='actuators_bringup',
            executable='actuators_generic',
            name='actuators_node'
        ),
        Node(
            package='sensors_bringup',
            executable='static_transform',
            name='static_transform_node'
        ),
        Node(
            package='sensors_bringup',
            executable='person_detect',
            name='person_detect_node'
        ),
        Node(
            package='head_bringup',
            executable='face',
            name='face_node'
        ),
        Node(
            package='face_tracker',
            executable='thermal_pose_simple',
            name='thermal_pose_node'
        ),
        Node(
            package='person_intent_classifier',
            executable='intent_node',
            name='intent_node'
        ),
        # Include the launch file for the radar node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(radar_vitals_launch_file),
            launch_arguments={
                'rviz': 'false',
                # Keep the old driver's frame name, so the camera-radar
                # calibration in static_transform still applies.
                'frame_id': 'ti_mmwave_0',
            }.items()
        ),
    ])
