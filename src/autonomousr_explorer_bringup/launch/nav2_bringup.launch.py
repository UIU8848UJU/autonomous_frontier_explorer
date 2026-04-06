from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory("nav2_bringup")
    this_pkg = get_package_share_directory("autonomousr_explorer_bringup")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(this_pkg, "config", "nav2_burger.yaml"),
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        nav2_launch,
    ])
