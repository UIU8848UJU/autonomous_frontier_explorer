from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import sys

_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.append(_LAUNCH_DIR)

from bringup_utils import resolve_map_file


def generate_launch_description():
    bringup_pkg = get_package_share_directory("autonomousr_explorer_bringup")

    sim_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "sim_bringup.launch.py")
        )
    )

    map_file = resolve_map_file(bringup_pkg)
    if os.path.exists(map_file):
        mode_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, "launch", "full_system_static.launch.py")
            )
        )
    else:
        mode_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, "launch", "full_system_slam.launch.py")
            )
        )

    return LaunchDescription([
        sim_slam,
        mode_launch,
    ])
