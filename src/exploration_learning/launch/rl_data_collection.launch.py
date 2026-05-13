import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("exploration_learning")
    params_file = os.path.join(pkg_share, "config", "rl_data_collector.yaml")

    return LaunchDescription([
        Node(
            package="exploration_learning",
            executable="rl_data_collector_node",
            name="rl_data_collector_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
