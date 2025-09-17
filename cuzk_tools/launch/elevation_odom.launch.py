from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='cuzk_tools',
            executable='elevation_odom',
            name='elevation_odom_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("cuzk_tools"), 'params', 'params.yaml')],
            remappings = [('odom', 'gps_odom')],
        ),
])