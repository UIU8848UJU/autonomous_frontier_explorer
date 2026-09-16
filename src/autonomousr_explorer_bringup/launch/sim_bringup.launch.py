import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_pkg = get_package_share_directory("autonomousr_explorer_bringup")
    gazebo_pkg = get_package_share_directory("gazebo_ros")
    turtlebot_pkg = get_package_share_directory("turtlebot3_gazebo")

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    world = os.path.join(bringup_pkg, "worlds", "turtlebot3_world_3x.world")

    # 使用仓库内 world 的物理参数，避免依赖当前 Gazebo 是否暴露动态物理服务。
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gzclient.launch.py")
        )
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_pkg, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_pkg, "launch", "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="是否让 ROS 节点使用 Gazebo 仿真时间。",
        ),
        DeclareLaunchArgument(
            "x_pose",
            default_value="-2.0",
            description="机器人初始 x 坐标。",
        ),
        DeclareLaunchArgument(
            "y_pose",
            default_value="-0.5",
            description="机器人初始 y 坐标。",
        ),
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_turtlebot,
    ])
