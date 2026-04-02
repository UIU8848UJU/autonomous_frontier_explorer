from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    task_pkg = get_package_share_directory("task_manager")
    task_params = os.path.join(task_pkg, "config", "task_manager.yaml")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(task_pkg, "launch", "sim_bringup.launch.py")
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(task_pkg, "launch", "nav2_bringup.launch.py")
        )
    )

    task_node = Node(
        package="task_manager",
        executable="task_manager_node",
        name="task_manager_node",
        output="screen",
        parameters=[task_params]
    )

    return LaunchDescription([
        sim_launch,

        TimerAction(
            period=5.0,
            actions=[nav2_launch]
        ),

        TimerAction(
            period=10.0,
            actions=[task_node]
        ),
    ])