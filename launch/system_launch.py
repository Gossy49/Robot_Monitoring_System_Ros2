from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab2_py_pkg',
            executable='state_monitor',
            name='state_monitor',
            parameters=['config/state_monitor.yaml']
        ),
        Node(
            package='lab2_py_pkg',
            executable='error_handler',
            name='error_handler',
            parameters=['config/error_handler.yaml']
        )
    ])
