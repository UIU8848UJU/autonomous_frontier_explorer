^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
frontier_explorer 包更新日志
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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
