from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    task_pkg = get_package_share_directory("task_manager")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(task_pkg, "launch", "tb3_navigation2_no_rviz.launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(task_pkg, "config", "nav2_burger.yaml"),
            "use_sim_time": "true",
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        nav2_launch,
    ])