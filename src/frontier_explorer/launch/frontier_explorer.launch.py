from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("frontier_explorer")
    params_file = os.path.join(pkg_share, "config", "frontier_explorer.yaml")

    explorer_node = Node(
        package="frontier_explorer",
        executable="frontier_explorer_node",
        name="frontier_explorer_node",
        output="screen",
        parameters=[params_file]
    )

    return LaunchDescription([
        explorer_node
    ])