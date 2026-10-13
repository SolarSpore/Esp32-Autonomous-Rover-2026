from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.26'),
        DeclareLaunchArgument('robot_port', default_value='4210'),
        DeclareLaunchArgument('wheel_separation', default_value='0.30'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.5'),
        DeclareLaunchArgument('send_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('cmd_vel_timeout', default_value='0.5'),

        Node(
            package='rover_motor_bridge',
            executable='rover_motor_bridge',
            name='rover_motor_bridge',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'robot_port': LaunchConfiguration('robot_port'),
                'wheel_separation': LaunchConfiguration('wheel_separation'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'send_rate_hz': LaunchConfiguration('send_rate_hz'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
            }],
        ),
    ])
