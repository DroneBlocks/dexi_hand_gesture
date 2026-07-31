import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('dexi_hand_gesture'),
        'config',
        'hand_gesture_params.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the parameter file',
        ),
        Node(
            package='dexi_hand_gesture',
            executable='hand_gesture_node.py',
            name='hand_gesture',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
