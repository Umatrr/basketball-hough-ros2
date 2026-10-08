"""Start the camera, detector, and display nodes with one parameter file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('basketball_cv'))
    default_params = package_share / 'config' / 'params.yaml'
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=str(default_params),
            description='YAML file containing parameters for all three nodes',
        ),
        Node(
            package='basketball_cv',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='basketball_cv',
            executable='detector_node',
            name='basketball_detector',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='basketball_cv',
            executable='display_node',
            name='basketball_display',
            output='screen',
            parameters=[params_file],
        ),
    ])
