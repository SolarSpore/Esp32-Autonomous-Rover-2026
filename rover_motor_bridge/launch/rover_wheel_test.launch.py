from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.26'),
        DeclareLaunchArgument('robot_port', default_value='4210'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),

        Node(
            package='rover_motor_bridge',
            executable='rover_wheel_test',
            name='rover_wheel_test',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'robot_port': LaunchConfiguration('robot_port'),
                'cmd_timeout': LaunchConfiguration('cmd_timeout'),
            }],
        ),
    ])
