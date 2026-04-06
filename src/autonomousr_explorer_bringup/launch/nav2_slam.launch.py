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

    params_file = os.path.join(this_pkg, "config", "nav2_exploration.yaml")
    slam_params_file = os.path.join(this_pkg, "config", "slam_toolbox.yaml")
    map_file = resolve_map_file(this_pkg)

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": "true",
            "params_file": params_file,
            "slam_params_file": slam_params_file,
            "slam": "True",
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        nav2_launch,
    ])
