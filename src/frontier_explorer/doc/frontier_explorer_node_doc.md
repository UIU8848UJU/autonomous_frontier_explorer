# Frontier Explorer 模块技术文档

## 1. 模块定位

`frontier_explorer` 位于 `src/frontier_explorer`，负责在 `/map`
提供的 OccupancyGrid 上寻找可探索边界，选择下一个目标点，并通过 Nav2
`navigate_to_pose` action 派发探索目标。

当前模块已经从早期的字符串策略切换，重构为：

```text
FrontierDetector
  -> FrontierPruner
  -> FrontierScorer
  -> FrontierSelector
  -> NavigateToPose
```

模块输出 `/exploration_state`，并提供 `/start_exploration`、
`/stop_exploration` 两个 Trigger 服务。

## 2. 依赖与接口

### 依赖

- ROS 2 Humble
- Nav2 `NavigateToPose`
- Nav2 `nav2_costmap_2d::Costmap2D`
- `nav_msgs/msg/OccupancyGrid`
- `nav_msgs/msg/Odometry`
- `robot_interfaces/msg/ExplorationState`
- `util_package` 中的 friendly logging

### 接口

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 订阅 | frontier 检测、unknown 语义和候选基础合法性判断。 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 订阅 | 可选 safety costmap 来源，用于 clearance 软评分，不作为 frontier 硬过滤。 |
| `/odom` | `nav_msgs/msg/Odometry` | 订阅 | 将机器人位置转换为地图栅格坐标。 |
| `/exploration_state` | `robot_interfaces/msg/ExplorationState` | 发布 | 发布 IDLE/RUNNING/STOPPED/COMPLETED/STUCK 等状态。 |
| `/start_exploration` | `std_srvs/srv/Trigger` | 服务 | 进入 RUNNING，允许定时器派发目标。 |
| `/stop_exploration` | `std_srvs/srv/Trigger` | 服务 | 进入 STOPPED，停止派发新目标。 |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Action Client | 向 Nav2 发送选中的 frontier goal。 |

## 3. 核心流程

`FrontierExplorerNode::explore_timer_callback()` 是主循环：

1. 仅在状态为 RUNNING 且当前没有 action goal 执行时继续。
2. 检查 `/map` 是否超时；超时则进入 STUCK，detail 为 `map_stale`。
3. 根据 `/odom` 和 `/map` 原点/分辨率计算机器人当前 `GridCell`。
4. `FrontierDetector` 从 `/map` 中检测 frontier cell 并聚类。
5. `FrontierSelector` 调用 pruner/scorer，使用 `/map` 生成候选，使用 global costmap 计算 clearance 软评分。
6. 将 goal grid 转换为 `PoseStamped`，发送 Nav2 `NavigateToPose`。
7. 根据 action feedback/result 更新状态、失败计数和黑名单。

当前节点会周期性输出 frontier cell 数量、raw cluster 数量和
`min_cluster_size`，用于判断小边界是否在 detector 阶段被发现。

## 4. 组件职责

### 4.1 CostmapAdapter

`CostmapAdapter` 是地图访问适配层，内部使用 Nav2 `Costmap2D`，统一处理：

- `OccupancyGrid` 到 `Costmap2D` 的 cost 转换；
- `worldToMap` / `mapToWorld` 坐标转换；
- free / unknown / obstacle 判断；
- frontier unknown 邻居判断；
- 候选点到最近障碍的 clearance 查询。

当前维护两类地图来源：

```text
/map
  -> unknown frontier 检测
  -> free / unknown / obstacle 基础语义
  -> 候选点硬约束

/global_costmap/costmap
  -> clearance_m 计算
  -> ClearanceScore 软评分
  -> 不直接删除 frontier 候选
```

注意：global costmap 通常包含 inflation layer，代价会比 `/map` 更保守。
因此它只进入评分体系，不作为 pruner 的一票否决条件，避免小边界 frontier 被过早过滤。

### 4.2 FrontierDetector

`FrontierDetector` 只负责发现和聚类：

- `detect_frontier_cells`：遍历 free cell，寻找邻域存在 unknown cell 的 frontier。
- `is_frontier_cell_safe`：用 `obstacle_search_radius_cells` 排除贴近障碍的 frontier cell。
- `cluster_frontiers`：使用 8 邻域 BFS 把 frontier cell 聚成 cluster，并计算 centroid。

注意：`FrontierDetector` **不再按 cluster size 丢弃小 cluster**。
小边界需要保留给后续兜底探索，因此 cluster size 判断交给后面的
`FrontierPruner` 和 `FrontierSelector`。

### 4.3 FrontierPruner

`FrontierPruner` 负责硬过滤和候选修复：

- cluster size 最小值过滤；
- goal blacklist / retry 过滤；
- cluster retry / cluster blacklist 过滤；
- last goal 过滤；
- `min_goal_distance_m` 过滤；
- goal 必须落在当前 map 的 free cell；
- centroid 不可用时，在 cluster 内寻找 fallback goal；
- 统计候选点局部窗口内的 `unknown_ratio`；
- 查询候选点的 `clearance_m`，供 scorer 做软评分。

局部窗口统计由：

```yaml
frontier_decision.candidate_unknown_margin_cells
```

控制。当前 `unknown_ratio` 不再作为硬过滤直接丢弃候选，而是交给
`UnknownRiskPenaltyScore` 做风险扣分。

`clearance_m` 已接入 `CostmapAdapter::distanceToNearestObstacle()`。
当 `use_global_costmap_for_safety: true` 且 global costmap 已就绪时，pruner 会把
候选点从 `/map` 栅格转换到世界坐标，再转换到 global costmap 栅格，计算最近障碍距离。
如果 global costmap 不可用，则回退到 `/map` adapter。

`clearance_m` 只作为候选事实数据写入 `FrontierCandidate`，不改变 pruner 的硬过滤边界。

### 4.4 FrontierScorer

`FrontierScorer` 只负责打分：

- 调用各个 score component；
- 生成分项分数；
- 根据 YAML 权重合成 `total_score`；
- 输出 `ScoredFrontierCandidate`。

当前总分结构为：

```text
total_score =
  weight_distance * distance_score
+ weight_cluster_size * cluster_size_score
+ weight_clearance * clearance_score
- weight_retry_penalty * retry_penalty
- weight_unknown_risk_penalty * unknown_risk_penalty
+ optional scores
```

已接入的 score component：

- `DistanceScore`
- `ClusterSizeScore`
- `RetryPenaltyScore`
- `UnknownRiskPenaltyScore`
- `ClearanceScore`，基于 `/global_costmap/costmap` 或 `/map` 的最近障碍距离做软评分
- `InformationGainScore`，当前以 `unknown_ratio` 作为轻量代理，默认未启用

### 4.5 FrontierSelector

`FrontierSelector` 是编排层和长期状态持有者：

- 调用 `FrontierPruner` 生成有效候选；
- 将候选分为正常候选和小 cluster 候选；
- 优先从正常候选中打分选择；
- 只有正常候选为空时，才从小 cluster 候选中选择；
- 维护 last goal、goal failed counts、goal blacklist、cluster failed counts、cluster blacklist。

小 cluster 延后选择由：

```yaml
frontier_decision.defer_small_clusters: true
frontier_decision.small_cluster_size_threshold: 3
```

控制。默认 `cluster_size < 3` 的候选不会参与正常竞争，只作为兜底目标。

## 5. 状态机

| 状态 | 触发条件 | 说明 |
| --- | --- | --- |
| IDLE | 节点启动默认状态 | 只维护订阅，不派发目标。 |
| RUNNING | `/start_exploration` 或目标成功后继续探索 | 允许定时器选择并发送目标。 |
| COMPLETED | 反馈中距离当前 goal 很近 | 表示当前 goal 到达，不代表全局探索完成。 |
| STOPPED | `/stop_exploration` 或 action 失败/取消 | 需要外部重新 start。 |
| STUCK | map 超时、进展停滞、连续无可用 frontier | detail 会记录具体原因。 |

当前 `result_callback()` 中 Nav2 goal 成功后会回到 RUNNING，使探索继续寻找下一个 frontier。

## 6. 参数参考

参数文件：

- `src/frontier_explorer/config/frontier_explorer.yaml`
- `src/autonomousr_explorer_bringup/config/frontier_explorer.yaml`

full system 实际使用的是 bringup 包下的配置。

| 参数 | 当前默认 | 说明 |
| --- | --- | --- |
| `explore_period_sec` | 3.0 | 探索定时器周期。 |
| `obstacle_search_radius_cells` | 1 | frontier cell 周围障碍检查半径。 |
| `min_frontier_cluster_size` | 1 | pruner 的最小 cluster size；保留小边界候选。 |
| `min_goal_distance_m` | 0.45 | 目标点离机器人过近时跳过，避免 Nav2 立即判定成功。 |
| `max_retry_count` | 2 | 单个 goal 失败达到阈值后加入黑名单。 |
| `frontier_decision.max_cluster_retry_count` | 3 | cluster 连续失败达到阈值后加入 cluster blacklist。 |
| `frontier_decision.defer_small_clusters` | true | 是否把小 cluster 延后到兜底阶段选择。 |
| `frontier_decision.small_cluster_size_threshold` | 3 | 小 cluster 阈值，低于该值时视为兜底候选。 |
| `frontier_decision.weight_distance` | 1.0 | 距离分权重，越高越偏向近目标。 |
| `frontier_decision.weight_cluster_size` | 1.0 | cluster size 分权重，越高越偏向大 frontier。 |
| `frontier_decision.weight_clearance` | 0.25 | clearance 分权重，越高越偏向远离障碍或高 cost 区的目标。 |
| `frontier_decision.enable_clearance_score` | true | 是否启用 clearance 软评分。 |
| `frontier_decision.weight_unknown_risk_penalty` | 1.0 | unknown risk 扣分权重。 |
| `frontier_decision.enable_unknown_risk_penalty` | true | 是否启用 unknown ratio 风险扣分。 |
| `frontier_decision.candidate_unknown_margin_cells` | 2 | 局部 unknown ratio 统计窗口半径。 |
| `frontier_decision.candidate_max_unknown_ratio` | 0.4 | unknown risk 开始扣分的阈值。 |
| `map_stale_timeout_ms` | 5000 | 地图长时间不更新时进入 STUCK。 |
| `max_frontier_failures` | 3 | 连续找不到目标后进入 STUCK。 |
| `edge_tolerance_m` | 0.3 | 判断机器人是否靠近 map 边缘。 |
| `map_topic` | `/map` | 用于 frontier 检测的 OccupancyGrid topic。 |
| `global_costmap_topic` | `/global_costmap/costmap` | 用于 clearance 评分的 global costmap topic。 |
| `use_global_costmap_for_safety` | true | 是否订阅 global costmap 并将其作为 clearance 评分来源；不会作为硬过滤。 |

## 7. 策略调参方向

更激进：

```yaml
weight_cluster_size: 1.5
weight_distance: 1.0
weight_unknown_risk_penalty: 0.8
candidate_max_unknown_ratio: 0.5
```

更保守：

```yaml
weight_cluster_size: 0.7
weight_distance: 1.0
weight_clearance: 0.4
enable_clearance_score: true
weight_unknown_risk_penalty: 2.0
candidate_max_unknown_ratio: 0.25
defer_small_clusters: true
small_cluster_size_threshold: 3
```

小边界完备性：

- 不建议把 `min_frontier_cluster_size` 重新拉得很高；
- 推荐保留 `min_frontier_cluster_size: 1`；
- 用 `defer_small_clusters` 把小边界延后，而不是删除。
- global costmap 不应用于硬过滤小边界，只通过 `clearance_score` 影响排序。

## 8. Nav2 调试结论

当前 full system 使用 `autonomousr_explorer_bringup/config/nav2_exploration.yaml`。
阶段性调试结论：

- DWB 在当前探索场景下容易抖动、原地调整或触发 recovery；
- RPP 更适合当前 frontier 目标跟踪，能更自然地对齐 path；
- 若 `min_goal_distance_m` 小于 Nav2 `xy_goal_tolerance`，目标可能刚发出就被判定成功，机器人看起来不动；
- `allow_unknown: false` 已用于禁止 planner 穿真正 unknown cell，但 path 仍可能贴近 inflation/cost 灰色区域。

速度链路：

```text
controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> turtlebot3_diff_drive
```

## 9. 调试命令

检查状态：

```bash
ros2 topic echo /exploration_state
ros2 topic echo /task_manager_state
```

检查 BT 是否进入 recovery：

```bash
ros2 topic echo /behavior_tree_log
```

检查控制器：

```bash
ros2 param get /controller_server FollowPath.plugin
ros2 topic info /cmd_vel_nav -v
ros2 topic echo /cmd_vel
```

检查 planner unknown 设置：

```bash
ros2 param get /planner_server GridBased.allow_unknown
ros2 param get /global_costmap/global_costmap track_unknown_space
```

检查 costmap 评分来源：

```bash
ros2 topic info /global_costmap/costmap -v
ros2 param get /frontier_explorer_node use_global_costmap_for_safety
ros2 param get /frontier_explorer_node frontier_decision.enable_clearance_score
ros2 param get /frontier_explorer_node frontier_decision.weight_clearance
```

## 10. 后续 TODO

- 将 information gain 从当前 `unknown_ratio` 代理升级为更稳定的窗口信息量估计。
- 为 scorer 输出增加调试日志或可视化，便于解释每次选点原因。
- 后续可增加可视化 marker，显示每个 frontier candidate 的 clearance、unknown risk 和 total score。
