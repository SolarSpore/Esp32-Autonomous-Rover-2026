import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rover_control'),
        'config',
        'steam_deck_controls.yaml',
    )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='rover_control',
            executable='rover_control_node',
            name='rover_control_node',
            output='screen',
            parameters=[config],
        ),
    ])
