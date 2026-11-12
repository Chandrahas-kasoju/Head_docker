from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('command_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('data_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('frame_id', default_value='radar_link'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz with the radar and breathing point clouds'),
        DeclareLaunchArgument('plot', default_value='false',
                              description='Show live breathing graphs; closing the window stops the radar'),
        Node(
            package='radar_vitals',
            executable='radar_vitals_node',
            name='radar_vitals',
            output='screen',
            parameters=[{
                'command_port': LaunchConfiguration('command_port'),
                'data_port': LaunchConfiguration('data_port'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('radar_vitals'), 'rviz', 'vitals.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='radar_vitals',
            executable='breathing_plot',
            condition=IfCondition(LaunchConfiguration('plot')),
            on_exit=Shutdown(),
        ),
    ])
