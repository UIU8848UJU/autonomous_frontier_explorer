from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory("autonomousr_explorer_bringup")
    task_pkg = get_package_share_directory("task_manager")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "nav2_localization.launch.py")
        )
    )

    rviz_config = os.path.join(task_pkg, "rviz", "slam.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    frontier_params = os.path.join(bringup_pkg, "config", "frontier_explorer.yaml")
    frontier_node = Node(
        package="frontier_explorer",
        executable="frontier_explorer_node",
        name="frontier_explorer_node",
        output="screen",
        parameters=[frontier_params],
    )

    task_params = os.path.join(bringup_pkg, "config", "task_manager.yaml")
    task_node = Node(
        package="task_manager",
        executable="task_manager_node",
        name="task_manager_node",
        output="screen",
        parameters=[task_params],
    )

    return LaunchDescription([
        TimerAction(period=5.0, actions=[nav2_launch]),
        TimerAction(period=8.0, actions=[rviz_node]),
        TimerAction(period=10.0, actions=[frontier_node]),
        TimerAction(period=12.0, actions=[task_node]),
    ])