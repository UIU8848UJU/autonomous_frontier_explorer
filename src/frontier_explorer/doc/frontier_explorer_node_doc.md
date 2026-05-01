# Frontier Explorer 模块技术文档

## 1. 模块定位

`frontier_explorer` 位于 `src/frontier_explorer`，负责在 `/map`
提供的 OccupancyGrid 上寻找可探索边界，并通过服务返回下一个 frontier goal。
导航派发和探索流程编排由 `ExplorationBtOrchestratorNode` / BT 负责。

当前模块已经拆成 ROS wrapper + 纯 C++ 能力类：

```text
FrontierExplorerNode
  -> FrontierGoalProvider
      -> FrontierDetector
      -> FrontierPruner
      -> FrontierScorer
      -> FrontierSelector

ExplorationBtOrchestratorNode
  -> ComputeNextFrontierGoal service
  -> NavigateToFrontier
  -> MarkFrontierFailed service
```

`FrontierExplorerNode` 输出 `/frontier_explorer/state` 和兼容旧系统的
`/exploration_state`，并提供 frontier 能力服务。旧的 `/start_exploration`、
`/stop_exploration` 仅保留为兼容控制面，不会触发内部导航循环。

## 2. 依赖与接口

### 依赖

- ROS 2 Humble
- Nav2 `nav2_costmap_2d::Costmap2D`
- `nav_msgs/msg/OccupancyGrid`
- TF2，查询 `map <- base_link` 机器人位姿
- `visualization_msgs/msg/MarkerArray`
- `robot_interfaces/msg/ExplorationState`

### 接口

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 订阅 | frontier 检测、unknown 语义和候选基础合法性判断。 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 订阅 | 可选 safety costmap 来源，用于 clearance 软评分，不作为 frontier 硬过滤。 |
| `map <- base_link` | TF | 查询 | 获取 map frame 下的机器人位姿，用于转换为地图栅格坐标。 |
| `/exploration_state` | `robot_interfaces/msg/ExplorationState` | 发布 | 发布 IDLE/RUNNING/STOPPED/COMPLETED/STUCK 等状态。 |
| `/frontier_explorer/state` | `robot_interfaces/msg/ExplorationState` | 发布 | frontier 能力节点状态。 |
| `/frontier/raw_markers` | `visualization_msgs/msg/MarkerArray` | 发布 | 原始 frontier cluster 点云。 |
| `/frontier/candidate_markers` | `visualization_msgs/msg/MarkerArray` | 发布 | pruner/scorer 后仍参与评分的候选点。 |
| `/frontier/scored_markers` | `visualization_msgs/msg/MarkerArray` | 发布 | Top 5 评分候选的简短文本。 |
| `/frontier/selected_marker` | `visualization_msgs/msg/MarkerArray` | 发布 | 本轮最终选中的目标箭头。 |
| `/frontier/blacklist_markers` | `visualization_msgs/msg/MarkerArray` | 发布 | 已进入 goal blacklist 的目标点。 |
| `/frontier/rejected_markers` | `visualization_msgs/msg/MarkerArray` | 发布 | 本轮 detector 发现但 selector 未能选出有效目标的 frontier。 |
| `/frontier_explorer_node/get_next_frontier_goal` | `robot_interfaces/srv/GetNextFrontierGoal` | 服务 | 请求下一个 frontier goal。 |
| `/frontier_explorer_node/mark_frontier_failed` | `robot_interfaces/srv/MarkFrontierFailed` | 服务 | 外部导航失败后通知能力层更新 retry / blacklist。 |
| `/frontier_explorer_node/clear_frontier_blacklist` | `robot_interfaces/srv/ClearFrontierBlacklist` | 服务 | 清空 frontier blacklist。 |
| `/frontier_explorer_node/get_exploration_state` | `robot_interfaces/srv/GetExplorationState` | 服务 | 查询 frontier 能力节点状态。 |
| `/start_exploration` | `std_srvs/srv/Trigger` | 服务 | 兼容旧控制面，只更新状态和清理 marker。 |
| `/stop_exploration` | `std_srvs/srv/Trigger` | 服务 | 兼容旧控制面，只更新状态和清理 marker。 |

## 3. 核心流程

`FrontierExplorerNode` 不再有探索导航主循环。核心能力入口是
`FrontierGoalProvider::compute_next_frontier_goal()`：

1. 检查 `/map`、costmap adapter 和 `map <- base_link` TF 是否可用。
2. 检查 `/map` 是否超时；超时则进入 STUCK，detail 为 `map_stale`。
3. 根据 TF 得到的 map frame 机器人位姿和 `/map` 原点/分辨率计算当前 `GridCell`。
4. `FrontierDetector` 从 `/map` 中检测 frontier cell 并聚类。
5. `FrontierSelector` 调用 pruner/scorer，使用 `/map` 生成候选，使用 global costmap 计算 clearance 软评分。
6. 返回候选点、Top 5 评分文本、blacklist 和 selected goal 所需的可视化快照。
7. 将 goal grid 转换为 `PoseStamped` 并填充服务响应。

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

selector 会保留最近一轮 `last_scored_candidates()`，供 marker 层显示评分候选。
同时它会在日志里输出：

- 最终选中的候选及 total score；
- distance、cluster size、clearance、retry、unknown risk 等分项；
- Top 3 候选摘要；
- 目标失败次数、retry 阈值和加入 blacklist 的原因。

因此，“为什么这个分数最高”和“为什么最终选它”主要看 selector 日志；
RViz 只保留 Top 5 的简短分数标签，避免文字盖住地图。

### 4.6 FrontierMarkerPublisher

`FrontierMarkerPublisher` 是 RViz 可视化层，只负责把已有中间结果转成
`visualization_msgs/msg/MarkerArray`。它不参与 frontier 检测、过滤、评分或选择，
也不会改变探索策略。

当前 marker topic 和语义如下：

| Topic | ns | 类型 | 颜色 | 说明 |
| --- | --- | --- | --- | --- |
| `/frontier/raw_markers` | `raw_frontiers` | `POINTS` | 蓝色 | detector 输出的原始 frontier cells。 |
| `/frontier/candidate_markers` | `candidates` | `SPHERE` | 青色 | pruner/scorer 后仍参与评分的候选点。 |
| `/frontier/scored_markers` | `scored_candidates` | `TEXT_VIEW_FACING` | 黄色；retry 高时紫色 | Top 5 候选的 `#rank score` 简短文本。 |
| `/frontier/selected_marker` | `selected_goal` | `ARROW` | 绿色 | 从机器人当前位置指向最终目标。 |
| `/frontier/blacklist_markers` | `blacklist` | `SPHERE` | 红色 | 已加入 goal blacklist 的目标点。 |
| `/frontier/rejected_markers` | `rejected_candidates` / `rejected_frontiers` | `SPHERE` / `POINTS` | 红色 | 本轮 detector 发现但 selector 未能选出有效目标的 frontier 或候选。 |

每次发布前都会发送 `DELETEALL`，避免 RViz 残留旧 marker。
`/start_exploration` 和 `/stop_exploration` 作为兼容服务会调用 `clearAll()` 清理所有 frontier marker。

注意：RViz 中看到的一串红色球如果来自 `/slam_toolbox/graph_visualization`，
那是 SLAM Toolbox pose graph，不是 `/frontier/blacklist_markers`。

## 5. 状态机

| 状态 | 触发条件 | 说明 |
| --- | --- | --- |
| IDLE | 节点启动默认状态 | 只维护订阅和能力接口。 |
| RUNNING | 兼容 start 服务、成功选点、失败事件记录或 blacklist 清理 | 表示能力节点可继续响应请求。 |
| COMPLETED | `get_next_frontier_goal` 判断无 frontier | 表示 frontier 能力层认为探索完成。 |
| STOPPED | 兼容 `/stop_exploration` | 停止状态由外部编排层解释。 |
| STUCK | map 超时、连续无可用 frontier 或失败目标越界 | detail 会记录具体原因。 |

Nav2 goal 成功、失败、取消的流程状态由 BT orchestrator 维护；
`FrontierExplorerNode` 只通过 `mark_frontier_failed` 接收失败事件。

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
| `frontier_decision.candidate_goal_inset_cells` | 2 | 将 frontier 候选目标沿目标到机器人方向向已知 free space 内缩的 cell 数，避免目标贴 unknown 边界。 |
| `frontier_decision.candidate_max_unknown_ratio` | 0.4 | unknown risk 开始扣分的阈值。 |
| `map_stale_timeout_ms` | 5000 | 地图长时间不更新时进入 STUCK。 |
| `max_frontier_failures` | 3 | 连续找不到目标后进入 STUCK。 |
| `edge_tolerance_m` | 0.3 | 判断机器人是否靠近 map 边缘。 |
| `map_topic` | `/map` | 用于 frontier 检测的 OccupancyGrid topic。 |
| `global_costmap_topic` | `/global_costmap/costmap` | 用于 clearance 评分的 global costmap topic。 |
| `use_global_costmap_for_safety` | true | 是否订阅 global costmap 并将其作为 clearance 评分来源；不会作为硬过滤。 |
| `global_frame` | `map` | frontier goal 和机器人位姿查询使用的全局坐标系。 |
| `robot_base_frame` | `base_link` | 机器人底盘坐标系。 |
| `robot_pose_timeout_ms` | 200 | 查询 `global_frame <- robot_base_frame` TF 的超时时间。 |
| `show_all_candidate_markers` | false | 是否在 RViz 中显示全部候选球；默认只显示最终选中候选，避免备选点被误认为残留目标。 |
| `frontier_decision.enable_reachability_filter` | true | 是否用 Nav2 `ComputePathToPose` 对评分靠前候选做可达性过滤。 |
| `frontier_decision.max_reachability_checks` | 6 | 每轮最多检查多少个评分靠前候选，控制 planner 负载。 |
| `frontier_decision.compute_path_to_pose_action` | `compute_path_to_pose` | Nav2 planner action 名称。 |
| `frontier_decision.reachability_server_timeout_ms` | 200 | 等待 planner action server 的超时时间。 |
| `frontier_decision.reachability_check_timeout_ms` | 500 | 单个候选规划检查超时时间。 |
| `frontier_decision.reachability_planner_id` | 空 | Nav2 planner_id；空字符串使用默认 planner。 |

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

检查 frontier marker：

```bash
ros2 topic list | grep /frontier
ros2 topic info /frontier/raw_markers -v
ros2 topic info /frontier/scored_markers -v
ros2 topic info /frontier/selected_marker -v
ros2 topic info /frontier/blacklist_markers -v
```

在 RViz 中添加 `MarkerArray` display，分别选择上述 `/frontier/...` topic。
这些 topic 使用 transient local QoS；如果 RViz 后启动，也能拿到最近一次 marker。

## 10. 后续 TODO

- 将 information gain 从当前 `unknown_ratio` 代理升级为更稳定的窗口信息量估计。
- 后续可以把 rejected candidate 拆成更细的拒绝原因 topic 或文本，但不建议默认全部打开，避免 RViz 过载。
- 如果需要更强解释性，可以增加一个低频 debug topic，发布完整候选评分表，替代 RViz 上的大量文本。
