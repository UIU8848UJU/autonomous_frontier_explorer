from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Task节点

def generate_launch_description():
    pkg_share = get_package_share_directory("task_manager")
    params_file = os.path.join(pkg_share, "config", "task_manager.yaml")

    task_manager_node = Node(
        package="task_manager",
        executable="task_manager_node",
        name="task_manager_node",
        output="screen",
        parameters=[params_file]
    )

    return LaunchDescription([
        task_manager_node
    ])