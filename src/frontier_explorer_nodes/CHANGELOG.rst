^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
frontier_explorer 包更新日志
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0
==========

Frontier Explorer 拆包
----------------------
- 将原 ``src/frontier_explorer`` 单包拆分为 ``frontier_explorer_core`` 与 ``frontier_explorer_nodes`` 两个 ROS 2 包。
- ``frontier_explorer_core`` 承载核心能力层，包括 ``costmap``、``detector``、``selector``、``scoring``、``types``、``geometry``、``reachability``、``utils`` 以及 ``FrontierGoalProvider``。
- ``frontier_explorer_nodes`` 承载 ROS 节点、BehaviorTree 插件、Nav2 reachability adapter、marker publisher、launch、config、rviz 和文档资源。
- 内部 include 路径统一迁移为 ``frontier_explorer_core/...`` 与 ``frontier_explorer_nodes/...``，移除旧的 ``core/...``、``nodes/...`` 等短路径依赖。
- ``frontier_explorer_nodes`` 通过 ``find_package(frontier_explorer_core REQUIRED)`` 链接核心库，形成 nodes 依赖 core 的单向依赖关系。
- BehaviorTree 插件库保留目标名 ``frontier_explorer_bt_nodes``，默认加载路径调整为 ``frontier_explorer_nodes`` 包下的 ``lib/libfrontier_explorer_bt_nodes.so``。

Launch 与 Bringup 适配
----------------------
- ``frontier_explorer_nodes`` 继承原 ``frontier_explorer`` 的 ``frontier_explorer_node``、``navigation_node``、``exploration_bt_orchestrator_node`` 三个可执行入口，运行时 node 名、topic 和 service 名保持兼容。
- 更新 ``autonomousr_explorer_bringup`` 与 ``task_manager`` 中引用 frontier 节点的 launch 文件，ROS package 名从 ``frontier_explorer`` 改为 ``frontier_explorer_nodes``。
- ``autonomousr_explorer_bringup/package.xml`` 的运行依赖同步改为 ``frontier_explorer_nodes``。
- 原 ``frontier_explorer`` 包目录已移除，资源和文档迁移到 ``frontier_explorer_nodes``。

Exploration Learning 骨架
-------------------------
- 新增 ``exploration_learning`` 包，先搭建 RL 数据采集相关目录和 CMake/package 基础结构。
- 新增 ``data_collection``、``reward``、``dataset`` 三个 include 子模块。
- 新增 ``RlDataCollectorNode``、``EpisodeRecorder``、``RewardCalculator``、``DatasetWriter`` 的最小可编译骨架。
- 新增 ``config/rl_data_collector.yaml`` 与 ``launch/rl_data_collection.launch.py``，为后续接入轨迹记录、reward 计算和 dataset 写入预留入口。


0.0.9(2026-05-10)
------------------
* 整理 ``core/selector`` 头文件层次，拆分为 ``candidates``、``filters``、``scoring``、``scoring/components`` 和 ``state``，降低后续 selector / scorer 扩展时的 include 混杂。
* 抽离 ``IFrontierCandidateScorer`` 评分接口，新增 ``ApproachGoalCandidate``、``RobotContext`` 和 ``FrontierScoringWeights`` 独立头文件，为后续 ML scorer 接入预留边界。
* 移除 frontier 评分权重/开关 ROS YAML 参数入口，评分权重改为 classic scorer 内部配置；保留 ``candidate_max_unknown_ratio`` 作为候选硬过滤参数。
* 抽离共享 ``FootprintCollisionChecker`` 几何组件，frontier 硬过滤阶段会先用 global costmap 检查候选目标 footprint 落脚安全；``NavigationNode`` 继续复用同一逻辑做执行前可行性兜底检查。
* 完善并启用 ``InformationGainScore``，classic scorer 默认将局部 unknown 密度和 frontier cluster 规模估计纳入总分。

0.0.8(2026-05-09)
------------------
* 修复 ``NavigationNode`` 使用 detached 线程执行导航导致的生命周期风险；导航执行线程现在由节点持有，节点析构时会请求停止、取消 active Nav2 goal 并 join 线程，避免 shutdown/restart 时线程继续访问已销毁的节点成员。

0.0.7 (2026-05-05)
-------------------
* 新增包级 ``README.md``，补充当前 BT-ready exploration 架构、关键接口、启动命令、状态 topic、marker topic 和已知现象。
* 在 README 顶部新增架构徽章，链接到 ``doc/exploration_architecture.md``。
* 更新 ``doc/exploration_architecture.md``，加入 Mermaid 架构图，明确 ``TaskManagerNode``、``ExplorationBtOrchestratorNode``、BT 插件、``FrontierExplorerNode``、``FrontierGoalProvider``、``NavigationNode`` 和 Nav2 之间的边界。
* 更新技术文档，记录当前商业落地风格的核心边界：frontier 能力层只生成候选和维护 retry / blacklist，BT 编排层负责流程，NavigationNode 负责导航可执行性和 Nav2 桥接。
* ``FrontierPruner`` 增加向机器人方向退避候选和环形采样候选，候选 yaw 默认朝向 frontier centroid，用于缓解 centroid 贴边或小毛刺导致无点可选的问题。
* 退避候选和环形采样候选增加去重，避免多个距离/角度落到同一个地图 cell 后重复参与评分。
* 将 fallback 采样层数收窄：退避距离保留 ``0.25``、``0.4``，环形半径保留 ``0.35``、``0.55``，减少末期 RViz 中候选点铺得过厚。
* ``FrontierPruner`` 增加 safety costmap 硬约束，候选点会映射到 ``/global_costmap/costmap``，并拒绝 costmap 外、unknown、障碍和 inflation 碰撞区的候选，避免退避/采样点落到障碍区域。
* ``FrontierGoalProvider`` 生成候选 ``PoseStamped`` 时使用 map frame，并将候选朝向设置为从 candidate 看向 frontier centroid。
* ``SelectFeasibleFrontier`` 不再简单选择第一个可执行候选，而是在检查窗口内综合 frontier score 和 path length 选择更优可执行目标，降低第一点过远或路径绕行过长的问题。
* ``NavigationNode`` 增加 path safety check，使用 global costmap 审计 Nav2 planner 返回 path 是否穿越 unknown、越界或高代价区域。
* ``NavigationNode`` 增加单目标执行边界：已有探索导航 goal 执行或取消中时，新 goal 会被拒绝，避免多个外部 goal 短时间抢占 Nav2 ``NavigateToPose``。
* 更新参数文档，记录 ``enable_path_safety_check``、``allow_unknown_path``、``path_cost_threshold``、``feasible_path_length_weight`` 等当前已接入参数。
* 记录当前已知现象：RViz 中 global path 在选点阶段可能短暂跳动，通常来自 BT 对多个候选调用 ``ComputePathToPose`` 做可执行性检查时 planner 临时 path 被显示，不一定代表真正导航 goal 被反复发送。
* 记录当前已知限制：探索末期仍可能残留一格左右 unknown，通常由小 cluster 延后、unknown ratio、goal inset、footprint/path safety 等保守策略共同导致；后续建议引入独立 cleanup exploration 收尾模式，而不是继续在主策略上叠加特殊规则。
* 记录当前策略风险：候选生成、fallback、可执行性过滤、path safety 与 blacklist 已形成较长策略链，后续需要拆分 normal exploration 与 cleanup exploration，降低调参互相影响。

0.0.6(2026-05-01)
------------------
* 将探索决策编排从 ``FrontierExplorerNode`` 中解耦，``FrontierExplorerNode`` 默认只作为 frontier 目标生成能力节点。
* 新增 ``FrontierGoalProvider`` 纯 C++ 能力类，承接地图/costmap、detector、pruner、scorer、selector、retry/blacklist 和 frontier goal 计算。
* 将 ``FrontierExplorerNode`` 收敛为 ROS wrapper，只负责参数、订阅、service、marker 和 state，不再持有 Nav2 action client。
* 将机器人位姿来源从 ``/odom`` 订阅改为 TF 查询 ``global_frame <- robot_base_frame``，避免把会漂移的 odom frame 坐标当作 map frame 使用。
* 新增 ``global_frame``、``robot_base_frame``、``robot_pose_timeout_ms`` 参数，默认分别为 ``map``、``base_link``、``200``。
* 新增 ``show_all_candidate_markers`` 参数，默认 ``false``；RViz 默认只显示最终选中候选球，避免完整候选集合在探索后期被误认为残留目标。
* 新增 frontier 可达性过滤：通过 Nav2 ``ComputePathToPose`` 检查评分靠前候选是否可规划路径，不可达候选不会被选为目标。
* 新增 ``frontier_decision.enable_reachability_filter``、``frontier_decision.max_reachability_checks``、``frontier_decision.compute_path_to_pose_action``、``frontier_decision.reachability_server_timeout_ms``、``frontier_decision.reachability_check_timeout_ms``、``frontier_decision.reachability_planner_id`` 参数。
* 新增 ``frontier_decision.candidate_goal_inset_cells`` 参数，候选目标会从 frontier 边界向机器人方向内缩到已知 free space，降低目标贴 unknown 边界导致 Nav2 拒绝的概率。
* 将 ``frontier_explorer_node`` executor 改为 ``MultiThreadedExecutor``，避免 service 回调中等待 Nav2 planner action 结果时阻塞 action 回调处理。
* 新增 ``robot_interfaces/srv/GetNextFrontierGoal``，用于外部请求下一个 frontier goal，并返回 success、reason、score、distance、clearance、frontier count、blacklist count、exploration_complete、recoverable 等字段。
* 新增 ``robot_interfaces/srv/MarkFrontierFailed``，用于导航失败后由外部编排层通知 frontier 能力节点更新 retry / blacklist。
* 新增 ``robot_interfaces/srv/ClearFrontierBlacklist``，用于清空 frontier blacklist。
* 新增 ``robot_interfaces/srv/GetExplorationState``，复用已有 ``ExplorationState`` 消息查询当前探索状态。
* 新增 ``FrontierGoalResult`` 和 ``FrontierGoalProvider::compute_next_frontier_goal()``，将 frontier 检测、过滤、打分和选择封装为纯能力接口，不直接触发 Nav2 goal。
* 废弃 ``enable_internal_navigation_loop`` 参数；即使配置为 ``true``，``FrontierExplorerNode`` 也不会主动发送 ``NavigateToPose``。
* 保留 retry / blacklist 管理在 ``FrontierGoalProvider`` 内部，外部 BT 只通过 ``MarkFrontierFailed`` 通知失败事件。
* 新增 ``ExplorationBtOrchestratorNode``，作为唯一探索编排层，加载 ``behavior_trees/exploration_tree.xml`` 并周期 tick BehaviorTree。
* 新增 BT 节点 ``ComputeNextFrontierGoal``、``NavigateToFrontier``、``MarkFrontierFailed``、``IsExplorationComplete``。
* 将 BT 节点拆成 BehaviorTree.CPP 动态插件库 ``libfrontier_explorer_bt_nodes.so``，orchestrator 通过 ``bt_plugin_libraries`` 参数加载插件。
* ``ExplorationBtContext`` 通过 BehaviorTree blackboard 注入插件节点，blackboard key 为 ``exploration_bt_context``，orchestrator 不再手写注册具体 BT 节点。
* 新增 ``robot_interfaces/srv/GetFrontierCandidates`` 和 ``FrontierGoalProvider::compute_frontier_candidates()``，为下一步 ``ComputeFrontierCandidates`` / ``SelectReachableFrontier`` BT 插件预留候选列表接口。
* 新增 ``robot_interfaces/action/NavigateToPose`` 和 ``navigation_node``，对 BT 暴露 ``/navigation_node/navigate_to_pose`` action，内部桥接 Nav2 ``NavigateToPose``。
* ``NavigateToFrontier`` BT 插件改为调用 ``NavigationNode`` action，不再直接依赖 ``nav2_msgs/action/NavigateToPose``，导航能力边界进一步收敛。
* 新增 ``navigation_action`` 参数；``navigate_to_pose_action`` 现在由 ``NavigationNode`` 内部用于配置 Nav2 action 名称。
* 新增 ``robot_interfaces/srv/CheckPoseReachability``，``navigation_node`` 内部通过 Nav2 ``ComputePathToPose`` 判断目标可达性并返回 path length。
* 新增 BT 插件 ``ComputeFrontierCandidates`` 和 ``SelectReachableFrontier``，默认 BT XML 改为先取候选列表、再由 BT 调用 NavigationNode 过滤可达目标。
* 新增 ``robot_interfaces/srv/CheckGoalFeasibility`` 和 BT 插件 ``SelectFeasibleFrontier``，在 planner 可达性前增加 goal pose footprint 落脚碰撞检查。
* ``navigation_node`` 新增 ``/navigation_node/check_goal_feasibility``，组合 ``/global_costmap/costmap`` footprint 检查与 Nav2 ``ComputePathToPose``，返回 feasible / reachable / footprint_valid。
* 默认 BT XML 从 ``SelectReachableFrontier`` 切换到 ``SelectFeasibleFrontier``，候选必须满足 footprint 落脚和 planner path 两类约束后才会导航。
* 默认关闭 ``frontier_decision.enable_reachability_filter``，避免 FrontierExplorerNode 和 BT 同时做可达性过滤；FrontierExplorerNode 只负责候选生成和 marker/state。
* 新增 ``frontier_candidates_service``、``reachability_service``、``goal_feasibility_service``、``max_frontier_candidates``、``check_pose_reachability_service``、``check_goal_feasibility_service``、``footprint_costmap_topic``、``enable_footprint_collision_check``、``allow_unknown_footprint``、``robot_radius``、``footprint_padding``、``footprint_cost_threshold``、``compute_path_to_pose_action``、``reachability_timeout_ms`` 等参数。
* 删除普通 C++ 状态机版 ``ExplorationOrchestratorNode``，避免 BT 与状态机两套编排逻辑并存。
* 更新 ``TaskManagerNode`` 配置，使探索启动/停止服务指向 ``/exploration_bt_orchestrator_node/start_exploration`` 和 ``/exploration_bt_orchestrator_node/stop_exploration``。
* 更新 bringup 和 sim launch，默认启动 ``exploration_bt_orchestrator_node``。
* 新增 ``/frontier_explorer/state`` 和 ``/exploration_orchestrator/state`` 职责区分；继续发布旧 ``/exploration_state`` 作为兼容 topic。
* 整理源码目录：公共头文件迁移到 ``include/frontier_explorer``，源码迁移到 ``src``；BT 编排相关源码集中在 ``src/nodes/bt``。
* 清理 CMake：新增 ``frontier_explorer_core`` library，抽出 ``configure_frontier_explorer_target`` 复用 include path 和 ament dependencies。
* 统一内部 include 风格，通过 CMake 暴露 ``include/frontier_explorer``，源码内部使用 ``nodes/...``、``core/...`` 等短路径。
* 新增 ``exploration_bt_defaults.hpp``，集中保存 BT orchestrator 的默认 service/action 名称和 tick 参数；生产部署仍通过 YAML 参数覆盖。
* 新增 ``doc/exploration_architecture.md`` 和 ``doc/exploration_bt_design.md``，记录当前架构、服务接口、BT XML、启动方式和后续 plugin 化方向。

0.0.5(2026-04-28)
------------------
* 新增 ``CostmapAdapter``，内部基于 Nav2 ``nav2_costmap_2d::Costmap2D`` 统一封装 OccupancyGrid 更新、world/map 坐标转换、cost 查询、free/unknown/obstacle 判断和 frontier unknown 邻居判断。
* ``FrontierDetector`` 改为通过 ``CostmapAdapter`` 访问地图，不再在 detector 中手写 OccupancyGrid 索引、边界检查和 unknown 邻居判断。
* ``FrontierPruner`` 改为接收 frontier map adapter 与可选 safety costmap adapter，保留 ``/map`` 作为 frontier 候选硬约束来源。
* 接入 ``/global_costmap/costmap`` 作为 clearance 评分来源：候选点先从 ``/map`` 栅格转换到世界坐标，再转换到 global costmap 栅格并查询最近障碍距离。
* 调整 global costmap 使用策略：global costmap 不作为 frontier 候选硬过滤条件，只通过 ``clearance_m`` 和 ``ClearanceScore`` 参与软评分，避免小边界 frontier 被 inflation/high cost 过早丢弃。
* 默认启用 ``frontier_decision.enable_clearance_score``，并将 ``frontier_decision.weight_clearance`` 设置为 0.25，用于轻量偏向更宽敞的目标。
* 新增 ``map_topic``、``global_costmap_topic``、``use_global_costmap_for_safety`` 参数，并同步更新 ``autonomousr_explorer_bringup`` 中实际使用的 frontier 配置。
* 为 detector、pruner、scorer、selector、costmap adapter 增加 ``rclcpp::Logger`` 成员和 child logger，便于定位地图转换、候选过滤和评分问题。
* 修复探索节点在非 RUNNING、正在导航或本轮无可用 frontier 时不发布 ``/exploration_state`` 的问题，避免 TaskManager 因心跳超时误判探索卡住。
* 新增 ``FrontierMarkerPublisher``，集中发布 RViz ``MarkerArray``，避免把 marker 生成逻辑写入 detector、pruner、scorer 或 selector。
* 新增 frontier 可视化 topic：``/frontier/raw_markers``、``/frontier/candidate_markers``、``/frontier/scored_markers``、``/frontier/selected_marker``、``/frontier/blacklist_markers``，并预留 ``/frontier/rejected_markers``。
* raw frontier 使用蓝色 ``POINTS``，候选点使用青色 ``SPHERE``，评分候选使用黄色文本，最终目标使用绿色 ``ARROW``，blacklist 使用红色 ``SPHERE``。
* 修正 selected goal 箭头方向，``/frontier/selected_marker`` 现在从机器人当前位置指向最终目标。
* ``/frontier/scored_markers`` 只显示 Top 5 候选的简短 ``#rank score`` 文本，详细评分拆解改由 selector 日志输出，降低 RViz 文字遮挡。
* selector 增加最终选点解释日志和 Top 3 候选摘要，包含 total score、distance、cluster、clearance、retry、unknown risk 等字段，便于解释为什么选中某个 frontier。
* blacklist 目标通过 ``/frontier/blacklist_markers`` 红色球显示，并在目标失败、达到 retry 阈值和加入 blacklist 时输出日志。
* ``/frontier/rejected_markers`` 用于显示本轮 detector 发现但 selector 未能选出有效目标的 frontier，发布到独立 marker topic，不占用 ``/map``。
* 更新 ``frontier_explorer_node_doc.md``，补充 ``/map`` 与 ``/global_costmap/costmap`` 的职责分工、soft costmap scoring、marker 可视化策略和相关调试命令。

0.0.4(2026-04-26)
------------------
* 将 frontier 选择链路重构为 ``FrontierPruner``、``FrontierScorer``、``score_components``、``FrontierSelector``。
* 删除旧的 ``frontier_selection_strategy`` 扩展路线，后续策略通过 score component 和 YAML 权重组合表达。
* 新增 ``DistanceScore``、``ClusterSizeScore``、``RetryPenaltyScore``、``UnknownRiskPenaltyScore``、``InformationGainScore``、``ClearanceScore`` 等普通 C++ 打分组件。
* ``unknown_ratio`` 不再作为硬过滤直接丢弃候选，改为通过 ``UnknownRiskPenaltyScore`` 参与扣分。
* ``FrontierDetector`` 不再按 cluster size 丢弃小边界，只负责 frontier cell 检测和 8 邻域聚类。
* ``FrontierPruner`` 负责 cluster size、retry、blacklist、last goal、min distance、centroid fallback、候选点地图合法性和局部 unknown window 统计。
* ``FrontierSelector`` 新增小 cluster 延后选择机制：正常候选存在时，小 cluster 不参与竞争；正常候选耗尽后再作为兜底目标。
* 新增 ``frontier_decision.defer_small_clusters`` 和 ``frontier_decision.small_cluster_size_threshold`` 参数。
* 调整默认参数，保留小边界探索能力：``min_frontier_cluster_size`` 默认为 1，``min_goal_distance_m`` 调整为 0.45。
* 保留了保守策略的调参数值，后续可通过解除 ``bringup`` 里 ``frontier_explorer.yaml`` 注释掉的参数来转换。
* 重构参数结构，新增 ``FrontierExplorerParams`` 及 runtime/pruner/scorer/selection 分组。
* 将地图工具函数迁移到 ``utils/map``，frontier 选择辅助工具迁移到 ``utils/frontier``。

0.0.3(2026-04-05)
------------------
* 添加了更好的黑名单机制，防止机器人在一个地方来回踱步，导致陷入局部死循环。
* 添加了完整的feedback，对于卡死，或者长时间目标没有变化重新选择 ``frontier``。
* 删除了一些没有必要的调试日志
* 将之前的日志改为了更友好更可调试的日志
* 加入了一些工具函数
* 修复了机器人因为map不更新，从而导卡死的问题。
* 修复了会因为map不更新在在空闲区域多个点来回走的问题。


0.0.2 (2026-04-04)
------------------
* 重构了frontier的代码，将 ``frontier_explorer_node`` 中的选择和计算解耦了出来。
* 新增了重试机制，重试次数限制。
* 改变了文件结构更易于后期维护。
* 添加了控制状态的机制，用于查询长时间停留的原因。
* ``centroid`` 不可达的时候，会选择第二个最优可达点。
* 增加了 ``frontier_detector`` 以及 ``frontier_selector``，分别将其独立了出来。
* 增加了在关键代码区增加了调试日志。
* 清理了一些老的依赖以及老的环境。
* 目前可以控制建模的开始和结束了。

0.0.1 (2026-04-02)
------------------
* 初始创建 ``frontier_explorer`` 独立 ROS 2 功能包。
* ``frontier_explorer_node`` 基础骨架搭建，拆分头文件与源文件。
* 新增 launch 文件与参数配置文件，支持作为独立节点启动。
* 接入 OccupancyGrid 地图订阅与 Nav2 ``NavigateToPose`` action client。
* 实现基础 frontier 检测、frontier 聚类与目标点选择流程。
* 增加 frontier 安全距离过滤，避免直接选择过近障碍区域。
* 增加导航中状态保护，避免在已有 goal 执行过程中重复发送新目标。
* 增加基础日志输出，覆盖地图接收、frontier 选择、目标发送与导航结果的流程。
* 完成探索节点的基础工程结构，便于后续接入 Gazebo + SLAM Toolbox + Nav2 工作流。
