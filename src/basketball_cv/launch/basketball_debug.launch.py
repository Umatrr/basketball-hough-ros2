"""Start the basketball system with score and mask debug windows enabled."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch all nodes with lower-rate, thresholded diagnostics."""

    package_share = Path(get_package_share_directory('basketball_cv'))
    default_params = package_share / 'config' / 'params.yaml'
    params_file = LaunchConfiguration('params_file')
    debug_log_period = LaunchConfiguration('debug_log_period')
    debug_hough_min_score = LaunchConfiguration('debug_hough_min_score')
    debug_mask_min_color_score = LaunchConfiguration(
        'debug_mask_min_color_score'
    )
    debug_display_fps = LaunchConfiguration('debug_display_fps')

    debug_detector = {
        'debug_enabled': True,
        'debug_log_period': debug_log_period,
    }
    debug_display = {
        'debug_enabled': True,
        'display_fps': debug_display_fps,
        'debug_hough_min_score': debug_hough_min_score,
        'debug_mask_min_color_score': debug_mask_min_color_score,
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=str(default_params),
            description='YAML file containing node parameters',
        ),
        DeclareLaunchArgument(
            'debug_log_period',
            default_value='0.5',
            description='Seconds between detector score log messages',
        ),
        DeclareLaunchArgument(
            'debug_hough_min_score',
            default_value='0.0',
            description='Minimum candidate total score to draw the circle',
        ),
        DeclareLaunchArgument(
            'debug_mask_min_color_score',
            default_value='0.0',
            description='Minimum color score to show the mask pixels',
        ),
        DeclareLaunchArgument(
            'debug_display_fps',
            default_value='10.0',
            description='Debug display refresh rate',
        ),
        Node(
            package='basketball_cv', executable='camera_node',
            name='camera_node', output='screen', parameters=[params_file],
        ),
        Node(
            package='basketball_cv', executable='detector_node',
            name='basketball_detector', output='screen',
            parameters=[params_file, debug_detector],
        ),
        Node(
            package='basketball_cv', executable='display_node',
            name='basketball_display', output='screen',
            parameters=[params_file, debug_display],
        ),
    ])
