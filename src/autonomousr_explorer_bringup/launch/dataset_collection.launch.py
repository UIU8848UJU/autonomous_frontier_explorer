from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory("autonomousr_explorer_bringup")
    full_system_launch = os.path.join(bringup_pkg, "launch", "full_system.launch.py")

    episode_id = LaunchConfiguration("episode_id")
    dataset_output_dir = LaunchConfiguration("dataset_output_dir")
    recorder_start_delay = LaunchConfiguration("recorder_start_delay")

    full_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(full_system_launch)
    )

    dataset_recorder = Node(
        package="exploration_learning",
        executable="dataset_recorder_node",
        name="dataset_recorder_node",
        output="screen",
        parameters=[{
            "episode_id": episode_id,
            "dataset_output_dir": dataset_output_dir,
            "record_map": True,
            "record_decision": True,
            "record_navigation_result": True,
            "flush_every_n_records": 1,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="turtlebot3_world",
            description="World name reserved for future dataset launch wiring.",
        ),
        DeclareLaunchArgument(
            "episode_id",
            default_value="manual_episode",
            description="Dataset episode id used as output subdirectory.",
        ),
        DeclareLaunchArgument(
            "dataset_output_dir",
            default_value="datasets/frontier_exploration/raw",
            description="Root directory for dataset records.",
        ),
        DeclareLaunchArgument(
            "spawn_x",
            default_value="0.0",
            description="Initial robot x reserved for future dataset launch wiring.",
        ),
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
            description="Initial robot y reserved for future dataset launch wiring.",
        ),
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="0.0",
            description="Initial robot yaw reserved for future dataset launch wiring.",
        ),
        DeclareLaunchArgument(
            "recorder_start_delay",
            default_value="15.0",
            description="Seconds to wait before starting DatasetRecorderNode.",
        ),
        full_system,
        TimerAction(period=recorder_start_delay, actions=[dataset_recorder]),
    ])
