^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
autonomous_frontier_explorer 项目更新日志
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2026-04-06
==========

Task/State 接口统一
--------------------
- 新增 `robot_interfaces` 协议包，定义 `ExplorationState`、`TaskManagerState`、`StartExploration`、`Explore` 等统一消息/服务/action。
- `frontier_explorer` 改为发布结构化 `ExplorationState`，包含时间戳、状态枚举和 detail 文本。
- `task_manager` 改为订阅该消息，维护 `TaskFlow` 上下文并通过 `TaskManagerState` 心跳发布 map_ready、运行标志和错误描述。

TaskManager 接线与参数化
------------------------
- `TaskManagerNode` 所有 topic/service 名称、心跳周期均支持 YAML 参数配置，并在 `config/task_manager.yaml` 中给出默认值。
- 订阅 `/exploration_state` 并调用 `TaskFlow::update_exploration_state`，三个服务回调统一通过 `TaskFlow` 处理并立即发布状态。
- 心跳 timer 定期发布 `TaskManagerState`，供监控/可视化使用。

Bringup 工程
------------
- 新增 `autonomousr_explorer_bringup` 包，集中提供 `sim_bringup`, `nav2_bringup`, `sim_slam_bringup`, `full_system.launch.py`。
- `full_system.launch.py` 串联 Gazebo → SLAM Toolbox → Nav2 → RViz → FrontierExplorer → TaskManager，便于一键启动完整探索流程。
- bringup 包携带 task_manager/frontend/nav2/slam 的参数副本，方便单包部署。

文档与 README
-------------
- README 描述更新，说明 `autonomousr_explorer_bringup` 的作用、统一接口依赖，并明确导航流程仍在 TODO。
- 新增 `doc/task_manager_tec_doc.md`，记录 TaskManager/TaskFlow 模块职责、状态机说明、接口以及后续扩展方向。

其它
----
- `config/task_manager.yaml` 同步了心跳与接口参数，同时保留原 waypoint 配置。
- 调整 CMake/package 依赖，确保 `task_manager`/`frontier_explorer` 依赖 `robot_interfaces`。



2026-04-03
==========
- 完成初版的MVP