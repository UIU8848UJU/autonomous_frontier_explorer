^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
task_manager 包更新日志
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2026-04-06 - v0.2.0
--------------------
- 新增 `core/task_flow/`，定义 `TaskManagerState`、`TaskContext` 与 `TaskFlow`，支撑 IDLE → STARTING_MAPPING → MAPPING → WAITING_MAP_SAVE → MAPPING_DONE → FAILED 的主流程。
- 全面重写 `TaskManagerNode`，拆分 node/core 目录，提供 `/start_mapping`、`/stop_all` 服务并发布 `/task_manager_state`。
- 引入 ROS 接口配置化（topic/service 名称、心跳间隔、探索状态超时），并增加标准 RCLCPP FSM 状态迁移日志。
- 自动管理 Frontier 策略节点：`/start_mapping` 成功后触发 `/start_exploration`，收到 `COMPLETED` 后等待地图保存，`stop_all`/心跳超时则调用 `/stop_exploration`。
- `TaskManagerNode` 现会将 `/exploration_state` 传入 `TaskFlow`，并在心跳超时时进入 `FAILED` 状态、记录 `last_error`。
