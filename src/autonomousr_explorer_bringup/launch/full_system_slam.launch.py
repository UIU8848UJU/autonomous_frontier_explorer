from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory("autonomousr_explorer_bringup")
    frontier_pkg = get_package_share_directory("exploration_nodes")
    exploration_bt_pkg = get_package_share_directory("exploration_bt")
    task_pkg = get_package_share_directory("task_manager")
    map_lifecycle_pkg = get_package_share_directory("map_lifecycle")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "nav2_slam.launch.py")
        )
    )

    rviz_config = os.path.join(bringup_pkg, "rviz", "slam.rviz")
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    frontier_params = os.path.join(frontier_pkg, "config", "frontier_strategy.yaml")
    frontier_node = Node(
        package="exploration_nodes",
        executable="frontier_strategy_node",
        name="frontier_strategy_node",
        output="screen",
        parameters=[frontier_params],
    )

    exploration_bt_params = os.path.join(exploration_bt_pkg, "config", "exploration_bt.yaml")
    exploration_bt_orchestrator_node = Node(
        package="exploration_bt",
        executable="exploration_bt_orchestrator_node",
        name="exploration_bt_orchestrator_node",
        output="screen",
        parameters=[exploration_bt_params],
    )

    navigation_node = Node(
        package="exploration_nodes",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[frontier_params],
    )

    task_params = os.path.join(task_pkg, "config", "task_manager.yaml")
    task_node = Node(
        package="task_manager",
        executable="task_manager_node",
        name="task_manager_node",
        output="screen",
        parameters=[task_params],
    )

    map_lifecycle_params = os.path.join(map_lifecycle_pkg, "config", "map_lifecycle.yaml")
    map_lifecycle_node = Node(
        package="map_lifecycle",
        executable="map_lifecycle_node",
        name="map_lifecycle_node",
        output="screen",
        parameters=[map_lifecycle_params],
    )

    return LaunchDescription([
        TimerAction(period=8.0, actions=[nav2_launch]),
        TimerAction(period=10.0, actions=[rviz_node]),
        TimerAction(period=12.0, actions=[frontier_node]),
        TimerAction(period=12.5, actions=[navigation_node]),
        TimerAction(period=13.0, actions=[exploration_bt_orchestrator_node]),
        TimerAction(period=14.0, actions=[task_node]),
        TimerAction(period=14.0, actions=[map_lifecycle_node]),
    ])
