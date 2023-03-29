from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_frame', default_value='tool0',
            description='Target frame name.'
        ),
        Node(
            package='am_vision',
            executable='tf_reader',
            name='tf_reader',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        )
    ])