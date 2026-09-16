from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory("autonomousr_explorer_bringup")
    frontier_pkg = get_package_share_directory("exploration_nodes")
    exploration_bt_pkg = get_package_share_directory("exploration_bt")
    task_pkg = get_package_share_directory("task_manager")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "nav2_localization.launch.py")
        )
    )

    rviz_config = os.path.join(bringup_pkg, "rviz", "slam.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    frontier_params = os.path.join(frontier_pkg, "config", "frontier_strategy.yaml")
    frontier_node = Node(
        package="exploration_nodes",
        executable="frontier_strategy_node",
        name="frontier_strategy_node",
        output="screen",
        parameters=[frontier_params],
    )

    exploration_bt_params = os.path.join(exploration_bt_pkg, "config", "exploration_bt.yaml")
    exploration_bt_orchestrator_node = Node(
        package="exploration_bt",
        executable="exploration_bt_orchestrator_node",
        name="exploration_bt_orchestrator_node",
        output="screen",
        parameters=[exploration_bt_params],
    )

    navigation_node = Node(
        package="exploration_nodes",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[frontier_params],
    )

    task_params = os.path.join(task_pkg, "config", "task_manager.yaml")
    task_node = Node(
        package="task_manager",
        executable="task_manager_node",
        name="task_manager_node",
        output="screen",
        parameters=[task_params],
    )

    readiness_gate = Node(
        package="autonomousr_explorer_bringup",
        executable="readiness_gate.py",
        prefix="python3",
        name="bringup_readiness_gate",
        output="screen",
        arguments=[
            "--topic", "/map",
            "--topic", "/global_costmap/costmap",
            "--service", "/frontier_strategy_node/get_frontier_candidates",
            "--service", "/navigation_node/check_goal_feasibility",
            "--service", "/lifecycle_manager_localization/manage_nodes",
            "--timeout-sec", "120.0",
        ],
    )

    launch_after_readiness = {"started": False}

    def start_runtime_nodes(event, _context):
        text = event.text.decode(errors="replace") if isinstance(event.text, bytes) else str(event.text)
        if launch_after_readiness["started"] or "READINESS_GATE_READY" not in text:
            return []
        launch_after_readiness["started"] = True
        return [exploration_bt_orchestrator_node, task_node]

    return LaunchDescription([
        nav2_launch,
        rviz_node,
        frontier_node,
        navigation_node,
        readiness_gate,
        RegisterEventHandler(
            OnProcessIO(
                target_action=readiness_gate,
                on_stdout=start_runtime_nodes,
            )
        ),
    ])
