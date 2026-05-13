^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
autonomous_frontier_explorer v0.1.0 发布说明
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

v0.1.0 - 2026-05-13
===================

版本定位
--------
- 完成 autonomous frontier exploration 的首个可运行版本，覆盖 Gazebo、SLAM Toolbox、Nav2、frontier 目标选择、BT 编排、TaskManager 状态管理和 RViz 调试可视化。
- 当前版本面向 ROS 2 Humble / Ubuntu 22.04 / Nav2 / SLAM Toolbox / TurtleBot3 仿真工作流。
- 演示场景已包含激进探索与保守探索两套 6 倍速运行记录，README 中保留视频入口。

包结构
------
- 新增 ``autonomousr_explorer_bringup``，集中维护 full system launch、Nav2、SLAM、frontier、task manager 和 RViz 配置。
- 新增 ``robot_interfaces``，统一定义探索状态、任务状态、服务和 action 协议。
- 新增 ``task_manager``，负责任务状态机、探索状态心跳、服务接线和上层流程控制。
- 将原 ``frontier_explorer`` 单包拆分为 ``frontier_explorer_core`` 与 ``frontier_explorer_nodes``。
- ``frontier_explorer_core`` 承载核心算法和共享类型，包括 ``costmap``、``detector``、``selector``、``scoring``、``types``、``geometry``、``reachability``、``utils`` 和 ``FrontierGoalProvider``。
- ``frontier_explorer_nodes`` 承载 ROS 节点、BehaviorTree 插件、Nav2 reachability adapter、marker publisher、launch、config、rviz 和文档资源。
- 新增 ``exploration_learning`` 骨架包，预留 ``data_collection``、``reward``、``dataset`` 三个模块，用于后续 RL 数据采集、reward 计算和 dataset 写入。

Frontier 探索能力
-----------------
- ``FrontierDetector`` 负责从 ``/map`` 中检测 unknown frontier 并完成聚类。
- ``FrontierPruner`` 负责候选硬过滤，包括 cluster size、last goal、blacklist、retry、最小距离、fallback goal 和候选点地图合法性。
- ``FrontierScorer`` 统一组合 distance、cluster size、retry penalty、unknown risk、information gain、clearance 等分项。
- ``FrontierSelector`` 维护长期选择状态，支持 retry / blacklist 策略和小 cluster 延后选择，降低碎片边界抢占目标的概率。
- 候选点必须落在当前 map 的 free cell 中，并可通过 ``candidate_unknown_margin_cells`` 和 ``candidate_max_unknown_ratio`` 限制候选周围 unknown 比例。
- ``FrontierGoalProvider`` 作为能力层入口，集中封装检测、过滤、打分、选择和失败反馈策略。

Costmap 与安全策略
------------------
- 新增 ``CostmapAdapter``，基于 Nav2 ``Costmap2D`` 统一处理 OccupancyGrid 更新、坐标转换、cost 查询和 frontier 基础判断。
- ``/map`` 负责 unknown frontier 检测和候选基础合法性判断。
- ``/global_costmap/costmap`` 作为 clearance 评分来源，不再作为 frontier 候选硬过滤条件。
- 候选点从 ``/map`` 栅格转换到世界坐标，再转换到 global costmap 栅格，用于估计到障碍或高风险区域的距离。
- ``ClearanceScore`` 默认启用，``weight_clearance`` 设置为 0.25，使更宽敞的候选目标具备轻微分数优势。

ROS 节点与 BT 编排
------------------
- ``frontier_explorer_node`` 作为 frontier 能力节点，提供 frontier goal、候选列表、失败标记、黑名单清理和探索状态查询服务。
- ``navigation_node`` 作为导航中间层，封装 reachability、feasibility 和导航 action 相关能力。
- ``exploration_bt_orchestrator_node`` 使用 BehaviorTree.CPP 编排探索流程。
- BT 动态插件库目标名为 ``frontier_explorer_bt_nodes``，默认加载路径为 ``frontier_explorer_nodes`` 包下的 ``lib/libfrontier_explorer_bt_nodes.so``。
- 节点运行时名称、topic 和 service 名称保持兼容，包括 ``/frontier_explorer_node/get_next_frontier_goal``、``/frontier_explorer_node/get_frontier_candidates``、``/frontier_explorer_node/mark_frontier_failed`` 和 ``/frontier_explorer/state``。

TaskManager 与统一接口
----------------------
- ``robot_interfaces`` 定义 ``ExplorationState``、``TaskManagerState``、``StartExploration``、``Explore`` 等统一协议。
- ``frontier_explorer_node`` 发布结构化 ``ExplorationState``，包含时间戳、状态枚举和 detail 文本。
- ``task_manager`` 订阅探索状态，维护 ``TaskFlow`` 上下文，并通过 ``TaskManagerState`` 心跳发布 map_ready、运行标志和错误描述。
- ``TaskManagerNode`` 的 topic、service 名称和心跳周期均支持 YAML 参数配置。
- 延长 TaskManager 调用探索相关 Trigger service 的等待时间，减少启动阶段误判失败。

Nav2 与运行配置
---------------
- full system launch 串联 Gazebo、SLAM Toolbox、Nav2、RViz、FrontierExplorer、NavigationNode、BT Orchestrator、TaskManager 和 MapManager。
- 探索模式 Nav2 控制器切换为 RPP（Regulated Pure Pursuit），用于降低 DWB 在 frontier 场景中的抖动和卡顿。
- 调整 ``min_goal_distance_m`` 与 Nav2 ``xy_goal_tolerance`` 的关系，避免过近目标导致 FollowPath 立即 SUCCESS。
- 探索配置中支持设置 ``GridBased.allow_unknown``，用于控制 planner 是否穿越 unknown cell。
- SLAM Toolbox 更新阈值降低，提高探索过程中的地图更新响应。

RViz 与调试
-----------
- 新增 ``FrontierMarkerPublisher``，集中发布 frontier 调试 marker。
- 新增 ``/frontier/raw_markers``、``/frontier/candidate_markers``、``/frontier/scored_markers``、``/frontier/selected_marker``、``/frontier/blacklist_markers`` 和 ``/frontier/rejected_markers``。
- raw frontier 使用蓝色 ``POINTS``，候选点使用青色 ``SPHERE``，评分候选使用黄色文本，最终目标使用绿色 ``ARROW``，blacklist 使用红色 ``SPHERE``。
- ``/frontier/scored_markers`` 只显示 Top 5 候选的简短 ``#rank score`` 文本，详细评分原因由 selector 日志输出。
- detector、pruner、scorer、selector 和 costmap adapter 增加 child logger，便于定位地图转换、候选过滤和评分问题。

工程与构建
----------
- 全部 C++ 包统一使用 C++17，并关闭编译器扩展。
- 内部 include 路径迁移为 ``frontier_explorer_core/...`` 与 ``frontier_explorer_nodes/...``。
- ``frontier_explorer_nodes`` 通过 ``find_package(frontier_explorer_core REQUIRED)`` 链接核心库，形成 nodes 依赖 core 的单向依赖关系。
- ``autonomousr_explorer_bringup`` 与 ``task_manager`` 中引用 frontier 节点的 launch 文件已改用 ``frontier_explorer_nodes`` 包名。
- ``autonomousr_explorer_bringup/package.xml`` 的运行依赖同步改为 ``frontier_explorer_nodes``。

验证记录
--------
- 已验证 ``colcon build --symlink-install --packages-select frontier_explorer_core frontier_explorer_nodes exploration_learning`` 通过。
- 已验证 ``colcon build --symlink-install --packages-select autonomousr_explorer_bringup task_manager frontier_explorer_core frontier_explorer_nodes exploration_learning`` 通过。
- 已完成基础仿真演示录制，包含激进探索与保守探索两种配置。

已知限制
--------
- ``exploration_learning`` 目前仅完成架构骨架，尚未接入真实 episode 记录、reward 回放和 dataset 落盘。
- 当前探索效果主要在 TurtleBot3 / Gazebo / SLAM Toolbox / Nav2 仿真链路中验证，真实机器人部署仍需要传感器、地图、控制器和安全参数复核。
- Frontier 决策仍以规则、过滤和加权打分为主，后续可继续接入学习式策略或更强的全局任务规划。
