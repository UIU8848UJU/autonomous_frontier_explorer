from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import sys

_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.append(_LAUNCH_DIR)

from bringup_utils import resolve_map_file


def generate_launch_description():
    nav2_pkg = get_package_share_directory("nav2_bringup")
    this_pkg = get_package_share_directory("autonomousr_explorer_bringup")
    frontier_pkg = get_package_share_directory("frontier_explorer")

    map_file = resolve_map_file(this_pkg)
    params_file = os.path.join(frontier_pkg, "config", "nav2_burger")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, "launch", "nav2_bringup.launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": "true",
            "params_file": params_file,
            "slam": "False",
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        nav2_launch,
    ])
