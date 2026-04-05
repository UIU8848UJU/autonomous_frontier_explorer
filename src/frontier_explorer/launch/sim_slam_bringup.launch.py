from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    task_pkg = get_package_share_directory("task_manager")
    frontier_pkg = get_package_share_directory("frontier_explorer")
    

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(task_pkg, "launch", "sim_bringup.launch.py")
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(frontier_pkg, "launch", "nav2_exploration_bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true"
        }.items()
    )
    rviz_config = os.path.join(frontier_pkg, "rviz", "slam.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config]
    )

    frontier_params = os.path.join(
        frontier_pkg, "config", "frontier_explorer.yaml"
    )

    explorer_node = Node(
        package="frontier_explorer",
        executable="frontier_explorer_node",
        name="frontier_explorer_node",
        output="screen",
        parameters=[frontier_params]
    )

    return LaunchDescription([
        sim_launch,
        TimerAction(period=5.0, actions=[slam_launch]),
        TimerAction(period=8.0, actions=[nav2_launch]),
        TimerAction(period=10.0, actions=[rviz_node]),
        TimerAction(period=15.0, actions=[explorer_node]),
    ])