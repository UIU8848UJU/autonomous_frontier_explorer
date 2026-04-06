# Task Manager Technical Notes

## 模块定位
- `task_manager` 是 Nav2 探索流程的调度中枢，用统一的状态机协调 `frontier_explorer`、导航与其他上层业务。
- `robot_interfaces` 中的消息定义为所有模块提供统一协议；`frontier_explorer` 按此协议发布 `ExplorationState`，`task_manager` 发布 `TaskManagerState`。
- `frontier_explorer` 仍然负责地图感知与目标选择；`task_manager` 不干预其内部算法，仅消费其状态并决定何时切换到导航或停止。

## 目录结构
- `config/`：参数文件（如 `task_manager.yaml`）控制话题、服务名称、心跳周期等。
- `core/task_flow/`：纯业务逻辑，描述 `TaskContext` 与状态流转。
- `core/task_state/` 与 `core/task_types/`：任务状态枚举、上下文结构。
- `doc/`：架构与维护文档（本文件）。
- `launch/`：示例启动脚本，读取 `config/task_manager.yaml`。
- `nodes/task_manager/`：ROS2 节点实现，负责通信接口与定时发布。
- `robot_interfaces/`（位于 workspace/src 根目录）：统一的 msg/srv/action 定义。

## 状态机说明
- `TaskManagerState`（见 `core/task_state/task_state.hpp`）枚举了 IDLE、EXPLORING、MAPPING_DONE、NAVIGATING 等阶段。
- `TaskContext`（`core/task_types/task_types.hpp`）记录地图是否可用、探索/导航运行标志、最近状态更新时间、最近探索状态文本与错误描述。
- `TaskFlow`（`core/task_flow/task_flow.*`）负责修改 `TaskContext`：
  - `start_mapping_flow()`：清理导航状态，进入 STARTING_EXPLORATION→EXPLORING。
  - `update_exploration_state()`：根据 `robot_interfaces/ExplorationState` 消息更新上下文、错误信息以及 TaskManager 状态。
  - `start_navigation_flow()`：在 `map_ready` 时进入 STARTING_NAVIGATION→NAVIGATING。
  - `stop_all()`：复位探索/导航运行标志并回到 IDLE。

## ROS 2 通信接口
- **订阅**：`TaskManagerNode` 订阅 `exploration_state_topic`（默认 `/exploration_state`，类型 `robot_interfaces/msg/ExplorationState`）。
- **发布**：定期发布 `task_manager_state_topic`（默认 `/task_manager_state`，类型 `robot_interfaces/msg/TaskManagerState`），包含枚举值与状态文本、地图/流程标志。
- **Service**：
  1. `start_mapping_service_name`（默认 `/start_mapping`，`std_srvs/Trigger`）
  2. `start_navigation_service_name`（默认 `/start_navigation`，`std_srvs/Trigger`）
  3. `stop_all_service_name`（默认 `/stop_all`，`std_srvs/Trigger`）
  服务仅调用 `TaskFlow`，完成后立即发布一次状态。
- **Timer**：`heartbeat_timer_` 周期性触发（参数 `heartbeat_period_ms`，默认 1000ms）进行状态心跳。

## 当前实现进度
- 第一版 `TaskFlow`+`TaskManagerNode` 完整接线：topic、service、timer 均按需求工作，状态心跳和服务回调统一走 `TaskFlow`。
- 关键参数已通过 YAML 配置（包含心跳周期以及所有 topic/service 名称），方便仿真/真机切换。
- 新的 `robot_interfaces` 包提供 Exploration/TaskManager 状态、StartExploration 服务及 Explore action 协议，`frontier_explorer` 已切换到新的状态消息。
- 文档化与示例配置齐备，launch 文件仍可复用原流程。

## 尚未覆盖的内容
- 尚未接入导航结果、teb/follow-up 行为反馈，也未实现完整的多任务/队列策略。
- `start_navigation` 仍由外部触发，尚未根据 `frontier_explorer` 自动切换。
- `robot_interfaces/srv` 与 action 仅定义，尚未融入现有节点。

## 后续扩展方向
1. **导航结果回调**：将 Nav2 action result 或 `frontier_explorer` 反馈映射到 TaskManager，触发重新探索或结束流程。
2. **多任务编排**：在 `TaskFlow` 中增加任务队列/优先级，与 future navigation/exploration 请求共存。
3. **Recovery/Fallback**：根据 `ExplorationState::STUCK` 或导航失败计数自动切换 recovery 流程。
4. **更细粒度参数**：把 cooldown、frontier 策略参数暴露到 YAML，并在 `TaskFlow` 里使用。
5. **Action 接口**：利用 `robot_interfaces/action/Explore` 对外提供长流程接口，统一 start/stop/result。

该文档可用于快速向新同事或面试者讲解当前实现，并指导下一阶段迭代。
