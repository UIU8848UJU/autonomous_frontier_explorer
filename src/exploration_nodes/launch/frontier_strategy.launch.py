from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("exploration_nodes")
    params_file = os.path.join(pkg_share, "config", "frontier_strategy.yaml")

    strategy_node = Node(
        package="exploration_nodes",
        executable="frontier_strategy_node",
        name="frontier_strategy_node",
        output="screen",
        parameters=[params_file]
    )

    return LaunchDescription([
        strategy_node
    ])
