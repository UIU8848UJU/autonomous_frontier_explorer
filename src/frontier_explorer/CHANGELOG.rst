^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
frontier_explorer 包更新日志
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2026-04-04)
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