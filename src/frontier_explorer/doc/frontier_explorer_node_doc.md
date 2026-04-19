# Frontier Explorer 模块技术文档

## 1. 模块定位与依赖
`frontier_explorer` 位于 `src/frontier_explorer`，是 Nav2 系统的自主探索插件，负责在全局 OccupancyGrid 地图上自动发掘导航目标。模块默认以 `frontier_explorer_node` 形式运行，依赖：
- ROS 2 Humble rclcpp、nav2_msgs、nav_msgs、geometry_msgs。
- Nav2 action `navigate_to_pose`（由 planner/controller/bt_navigator 等节点提供）。
- 地图话题 `/map`（nav_msgs/msg/OccupancyGrid，通常由 SLAM 映射节点提供）。
- 里程计话题 `/odom`（nav_msgs/msg/Odometry，来自底盘或仿真）。

模块输出探索状态话题 `/exploration_state`，同时暴露 `/start_exploration`、`/stop_exploration` 两个 Trigger 型服务用于外部控制。

## 2. 运行方式
1. 启动 Nav2 基础能力，可直接使用 `launch/nav2_exploration_bringup.launch.py` 来拉起 planner/controller/BT 等导航进程（配置文件 `config/nav2_exploration.yaml`）。
2. 通过 `launch/frontier_explorer.launch.py` 加载 `frontier_explorer_node`，默认参数文件 `config/frontier_explorer.yaml`。
3. 等待 `/map`、`/odom` 输入后，调用 `/start_exploration` 服务，节点进入 RUNNING 状态并按周期自动派发前沿目标；`/stop_exploration` 会令状态进入 STOPPED。

## 3. 重要节点接口
| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/map` | nav_msgs/msg/OccupancyGrid | 订阅 | 探索所用静态/半静态地图；QoS 为 `KeepLast=1 + transient_local` 以获取最新地图快照。 |
| `/odom` | nav_msgs/msg/Odometry | 订阅 | 将位置转换为栅格索引，参与前沿选择。 |
| `/exploration_state` | robot_interfaces/msg/ExplorationState | 发布 | 状态机输出，包含枚举值（IDLE/RUNNING/STOPPED/COMPLETED/STUCK）与 detail 描述。 |
| `/start_exploration` | std_srvs/srv/Trigger | 服务 | 将 `state_` 切换为 RUNNING，允许定时器开始发目标。 |
| `/stop_exploration` | std_srvs/srv/Trigger | 服务 | 强制状态 STOPPED，停止发布新目标。 |
| `navigate_to_pose` | nav2_msgs/action/NavigateToPose | Action Client | 将筛选出的前沿格点转换成 Pose 并发送，监听反馈/结果以更新状态与失败计数。 |

## 4. 核心组件
### 4.1 FrontierDetector（`core/frontier_detector`）
- `detect_frontier_cells`：遍历自由栅格，寻找周围存在未知（-1）单元且当前格安全的点。安全判定由 `is_frontier_cell_safe` 完成，检查 `obstacle_search_radius_cells` 范围内是否有障碍。
- `cluster_frontiers`：对前沿点进行 8 邻域 BFS 聚类，剔除小于 `min_frontier_cluster_size` 的簇，并计算几何中心（确保中心点也安全）。输出 `FrontierCluster` 列表供选择器使用。

### 4.2 FrontierSelector（`core/frontier_selector`）
- 依据机器人栅格坐标、地图分辨率与 `min_goal_distance_m` 过滤离机器人过近的目标。
- 维护失败计数与黑名单：若同一格失败次数超过 `max_retry_count`，或被标记为最近目标，则跳过。
- 若簇中心无效，调用 `find_fallback_goal_in_cluster` 在簇内寻找距离合适、未拉黑的替代格。
- `mark_cluster_failed/succeeded` 与 `cluster_blacklist_` 用于处理整个簇多次失败导致的局部循环。

### 4.3 FrontierExplorerNode（`nodes/frontier_explorer`）
- 初始化：声明参数 → 创建话题/服务/Action 客户端 → 创建 `explore_timer_`（周期默认 3s）。
- `map_callback`、`odom_callback`：缓存最新消息并记录 `last_map_update_time_`，用于陈旧地图检测。
- `explore_timer_callback` 流程：
  1. 仅在状态 RUNNING 且没有正在执行的 action 时继续。
  2. 检查地图是否超时（`map_stale_timeout_ms`），若超时则置 STUCK 并输出 detail `map_stale`。
  3. 调用 `update_robot_grid_position`，确认机器人仍在地图范围；否则跳过本周期。
  4. 调用 detector 检测前沿与聚类，selector 进行目标筛选。
  5. 若连续 `max_frontier_failures` 次没有可行目标，根据自身是否靠近地图边缘（`edge_tolerance_m`）设置 STUCK 原因。
  6. 将选中栅格转换成世界系目标（`map_utils::grid_to_goal_pose`），构造 NavigateToPose 目标发送。
- Action 反馈：
  - `feedback_callback` 读取 Nav2 回传的 `current_pose`，计算相对初始距离的进度。若 5s 内距离变化 <1cm，判定 STUCK 触发 fallback。
  - `result_callback` 根据 Nav2 返回的 ResultCode 更新状态及黑名单；SUCCEEDED 会调用 `mark_goal_succeeded`，其余结果标记失败并将状态置 STOPPED。

## 5. 状态机与容错
| 状态 | 触发条件 | 备注 |
| --- | --- | --- |
| IDLE | 节点启动默认状态或完成后重置 | 只维护订阅，不派发目标。 |
| RUNNING | `/start_exploration` 调用、或 Nav2 反馈仍在推进 | 允许定时器计算并发送目标。 |
| COMPLETED | Action 成功、或距离目标 < 5cm | 通常意味着当前目标到达，但探索不一定完成，下一周期会寻找新前沿。 |
| STOPPED | `/stop_exploration`、Action ABORT/CANCEL/未知错误 | 需要外部重新 `/start_exploration` 才会继续。 |
| STUCK | 长时间无地图更新、Action 进度停滞、或连续找不到前沿 | `state_detail` 会记录 `map_stale` / `no_valid_frontier` / `no_frontier_near_edge` 等，便于调试。 |

当 STUCK 由“连续无前沿”触发时，如果机器人靠近地图边缘（`edge_tolerance_m` 以内），即便没有新目标也会带 detail `no_frontier_near_edge`，提醒可能需要移动机器人或扩展地图。

## 6. 参数参考（`config/frontier_explorer.yaml`）
| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `explore_period_sec` | 3.0 | 探索定时器周期，越小越频繁地重新计算前沿。 |
| `obstacle_search_radius_cells` | 2 | 前沿安全检查的障碍半径（单位：栅格）。 |
| `min_frontier_cluster_size` | 5 | 小于此阈值的簇会被忽略，避免噪声。 |
| `min_goal_distance_m` | 0.5 | 筛除距离机器人太近的目标，防止重复振荡。 |
| `max_retry_count` | 2 | 同一目标失败次数达到该值后加入黑名单。 |
| `map_stale_timeout_ms` | 5000 | 若超过该时间未收到地图更新，标记 STUCK。 |
| `max_frontier_failures` | 3 | 连续几次无法获取有效前沿后触发 STUCK。 |
| `edge_tolerance_m` | 0.3 | 计算是否靠近地图边缘的公差，协助诊断。 |
| `use_sim_time` | true | 仅出现在参数文件中，用于仿真。 |

参数可通过 ROS 2 动态重载（例如 `ros2 param set`）调整，或创建新的 YAML 覆盖默认值。

## 7. 数据转换与工具函数（`core/map_utils`）
- `world_to_grid` / `grid_to_world`：在 OccupancyGrid 原点、分辨率下相互转换；超出范围时返回 `std::optional` 空值。
- `grid_to_goal_pose`：将格点转换为 `PoseStamped` 并对齐朝向，供 Nav2 Action 使用。
- `distance_in_meters`：用于反馈阶段计算进度。

这些工具函数保证所有几何计算都在单一坐标系下进行，避免 TF 依赖，适合轻量部署。

## 8. 调试与扩展建议
- **地图质量**：确保 `/map` 启用 `transient_local`，否则节点初次启动会迟迟收不到地图并保持 STUCK。
- **前沿参数调优**：
  - 探索缓慢时，可降低 `explore_period_sec` 或 `min_frontier_cluster_size`。
  - 若误判障碍导致无法靠近未知区域，放宽 `obstacle_search_radius_cells` 或调节 SLAM 的膨胀层。 
- **多机器人/调度**：当前实现只维护一个 action goal，可在 `FrontierExplorerNode` 中扩展为共享状态或分布式任务分配。
- **监控**：订阅 `/exploration_state`，结合 detail 字段即可构建简单 UI，判断何时人工干预。
- **日志**：使用 `friendly_logging` 宏输出，支持 `RCLCPP_*_WITH_CONTEXT`，可在 launch 中配置 `RCUTILS_CONSOLE_OUTPUT_FORMAT` 方便排查。

## 9. 快速验证流程
1. `colcon build --packages-select frontier_explorer && source install/setup.bash`。
2. `ros2 launch frontier_explorer nav2_exploration_bringup.launch.py use_sim_time:=true` 启动 Nav2。
3. `ros2 launch frontier_explorer frontier_explorer.launch.py` 启动探索节点。
4. `ros2 service call /start_exploration std_srvs/srv/Trigger {}` 进入 RUNNING。
5. 用 `ros2 topic echo /exploration_state` 观察状态变化，确认能周期性发送 `navigate_to_pose` 目标。

如需复位，可 `ros2 service call /stop_exploration std_srvs/srv/Trigger {}`，再重新 start。
