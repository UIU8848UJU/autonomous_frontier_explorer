from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import sys

_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.append(_LAUNCH_DIR)

from bringup_utils import resolve_map_file


def _resolve_mode_launch(context, bringup_pkg):
    mode = context.launch_configurations.get("mode", "auto")
    static_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "full_system_static.launch.py")
        )
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "full_system_slam.launch.py")
        )
    )
    if mode == "static":
        return [static_launch]
    if mode == "slam":
        return [slam_launch]
    map_file = resolve_map_file(bringup_pkg)
    if os.path.exists(map_file):
        return [static_launch]
    return [slam_launch]


def generate_launch_description():
    bringup_pkg = get_package_share_directory("autonomousr_explorer_bringup")

    sim_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "sim_bringup.launch.py")
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="auto",
            description="Exploration mode: auto (by map file), slam, or static.",
        ),
        sim_slam,
        OpaqueFunction(function=_resolve_mode_launch, args=[bringup_pkg]),
    ])
