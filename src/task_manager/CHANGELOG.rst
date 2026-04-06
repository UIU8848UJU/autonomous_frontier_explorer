^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
task_manager 包更新日志
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2026-04-06 - v0.2.0
--------------------
- 新增 `core/task_flow/`，定义 `TaskManagerState`、`TaskContext` 与 `TaskFlow`，支撑 IDLE → EXPLORING → MAPPING_DONE → NAVIGATING → FAILED 的状态机骨架。
- 全面重写 `TaskManagerNode`，拆分 node/core 目录，提供 `/start_mapping`、`/start_navigation`、`/stop_all` 服务并发布 `/task_manager_state`。
- 引入 ROS 接口配置化（topic/service 名称、心跳间隔、探索状态超时），并通过 `friendly_logging` 统一日志上下文。
- 自动管理 Frontier Explorer：`/start_mapping` 成功后触发 `/start_exploration`，收到 `COMPLETED` 会自动重新启动探索，`stop_all`/心跳超时则调用 `/stop_exploration`。
- `TaskManagerNode` 现会将 `/exploration_state` 传入 `TaskFlow`，并在心跳超时时进入 `FAILED` 状态、记录 `last_error`。
