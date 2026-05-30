from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("chassis_bridge"), "config", "real_chassis.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="chassis_bridge",
                executable="chassis_bridge_node",
                name="chassis_bridge_node",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
