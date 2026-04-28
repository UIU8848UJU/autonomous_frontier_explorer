^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
frontier_explorer 包更新日志
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
