from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("frontier_explorer")
    params_file = os.path.join(pkg_share, "config", "frontier_explorer.yaml")

    bt_orchestrator_node = Node(
        package="frontier_explorer",
        executable="exploration_bt_orchestrator_node",
        name="exploration_bt_orchestrator_node",
        output="screen",
        parameters=[params_file],
    )

    navigation_node = Node(
        package="frontier_explorer",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        navigation_node,
        bt_orchestrator_node,
    ])
