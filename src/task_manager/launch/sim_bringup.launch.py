from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# 拉起Gazebo，机器人模型
def generate_launch_description():
    tb3_gazebo_pkg = get_package_share_directory("turtlebot3_gazebo")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, "launch", "turtlebot3_world.launch.py")
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        gazebo_launch,
    ])