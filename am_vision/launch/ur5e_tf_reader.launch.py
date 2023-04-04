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
        DeclareLaunchArgument(
            'tcp_name', default_value='roscam_grip'
        ),
        DeclareLaunchArgument(
            'tcp_def', default_value='0,-.10318,.09253'
        ),
        Node(
            package='am_vision',
            executable='tf_reader',
            name='tf_reader',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')},
                {'tcp_name': LaunchConfiguration('tcp_name')},
                {'tcp_def': LaunchConfiguration('tcp_def')}
            ]
        )
    ])