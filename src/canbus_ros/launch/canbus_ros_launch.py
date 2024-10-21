import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('canbus_ros'),
        'config',
        'canbus_ros_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='canbus_ros',
            executable='motor_canbus',
            name='motor_canbus',
            output='screen',
            parameters=[config],
        ),
    ])
