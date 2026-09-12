from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bt_pkg_share = get_package_share_directory("exploration_bt")
    ros_pkg_share = get_package_share_directory("exploration_nodes")
    bt_params_file = os.path.join(bt_pkg_share, "config", "exploration_bt.yaml")
    ros_params_file = os.path.join(ros_pkg_share, "config", "frontier_strategy.yaml")

    bt_orchestrator_node = Node(
        package="exploration_bt",
        executable="exploration_bt_orchestrator_node",
        name="exploration_bt_orchestrator_node",
        output="screen",
        parameters=[bt_params_file],
    )

    navigation_node = Node(
        package="exploration_nodes",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[ros_params_file],
    )

    return LaunchDescription([
        navigation_node,
        bt_orchestrator_node,
    ])
