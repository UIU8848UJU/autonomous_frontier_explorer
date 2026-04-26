^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
autonomous_frontier_explorer 项目更新日志
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2026-04-26
==========

阶段性探索成果
--------------
- 顶层 README 重写为阶段性项目首页，保留原有徽章并新增 frontier_explorer 文档入口徽章。
- 新增 ``media/激进探索.mp4`` 与 ``media/保守探索.mp4`` 两个 6 倍速演示入口。
- README 明确当前演示采用 RPP 控制器，并说明 DWB 在当前场景下更容易抖动、停滞或依赖决策兜底。

Frontier 决策演进
-----------------
- 删除旧 ``frontier_selection_strategy`` 扩展路线，策略改由 score component 和 YAML 权重组合表达。
- ``FrontierDetector`` 只负责检测和聚类，不再在 detector 阶段丢弃小 cluster。
- ``FrontierPruner`` 负责硬过滤和候选修复，并保留局部 unknown window 统计。
- ``FrontierScorer`` 统一组合 distance、cluster size、retry penalty、unknown risk、information gain、clearance 等分项。
- ``FrontierSelector`` 新增小 cluster 延后选择机制，避免碎片小边界在正常候选存在时抢先被选。
- 新增 ``frontier_decision.defer_small_clusters`` 与 ``frontier_decision.small_cluster_size_threshold`` 参数。
- 当前 ``clearance_score`` 结构已接入，但 ``clearance_m`` 仍未接入真实 costmap/最近障碍距离统计。

Nav2 与运行配置
---------------
- 探索模式 Nav2 控制器切换为 RPP（Regulated Pure Pursuit），用于降低 DWB 在 frontier 场景中的抖动和卡顿。
- 调整 ``min_goal_distance_m`` 与 Nav2 ``xy_goal_tolerance`` 的关系，避免目标过近导致 FollowPath 立即 SUCCESS、机器人不移动。
- ``GridBased.allow_unknown`` 在探索配置中可设置为 false，用于禁止 planner 穿越真正 unknown cell；灰色 inflation/cost 区仍可能被规划器贴边经过。

2026-04-24
==========

Frontier 决策系统重构
---------------------
- 将 frontier 目标选择从字符串策略切换，重构为 Pruning、Scoring、Selection 三段式流程。
- 新增 `FrontierPruner`，集中处理 cluster size、last goal、blacklist、retry、最小距离、fallback goal、候选点地图合法性等硬过滤。
- 新增 `FrontierScorer`，通过 YAML 权重统一计算 distance、cluster size、clearance、revisit、retry 等分项分数和总分。
- `FrontierSelector` 改为持有长期状态，并通过 pruner/scorer 选择总分最高的候选目标。
- 旧 `frontier_selection_strategy.*` 后续不再作为主要扩展点，并在后续重构中删除。

探索候选点约束
--------------
- 候选点必须位于当前 map 的 free cell 中，避免把 goal 放进未知区或障碍区。
- 新增 `frontier_decision.candidate_unknown_margin_cells` 和 `frontier_decision.candidate_max_unknown_ratio` 参数，用于限制候选点周围未知区域比例。
- `FrontierExplorerNode` 在选择 frontier 时传入当前 `/map`，使候选过滤能够基于最新 OccupancyGrid 执行。

Bringup 与导航配置
------------------
- 明确 `full_system.launch.py` 使用 `autonomousr_explorer_bringup/config` 下的 Nav2、SLAM、FrontierExplorer 参数。
- 清理容易混淆的旧配置入口，将不再直接使用的 YAML 备份为 `.bak`。
- 调整探索模式 Nav2 参数，降低控制震荡和贴边卡死概率，包括 controller/planner 频率、DWB critic 权重、costmap 尺寸与 inflation 参数。
- 调整 SLAM Toolbox 更新阈值，降低 `minimum_travel_distance`、`minimum_travel_heading` 和 `minimum_time_interval`，提升探索过程中的地图更新响应。

TaskManager 稳定性
------------------
- 延长 TaskManager 调用探索相关 Trigger service 的等待时间，避免 `/start_mapping` 触发 `/start_exploration` 时因 1 秒超时误判失败。

文档与构建
----------
- README 新增 ROS 2 Humble、Ubuntu 22.04、C++17、Nav2、SLAM Toolbox、Gazebo、colcon、ament_cmake、开发状态等徽章。
- `frontier_explorer` 与 `task_manager` 的 CMake 构建标准统一为 C++17，并关闭编译器扩展以保证环境一致性。


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
