^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
frontier_explorer 包更新日志
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2026-04-02)
------------------
* 初始创建 ``frontier_explorer`` 独立 ROS 2 功能包。
* ``frontier_explorer_node`` 基础骨架搭建，拆分头文件与源文件。
* 新增 launch 文件与参数配置文件，支持作为独立节点启动。
* 接入 OccupancyGrid 地图订阅与 Nav2 ``NavigateToPose`` action client。
* 实现基础 frontier 检测、frontier 聚类与目标点选择流程。
* 增加 frontier 安全距离过滤，避免直接选择过近障碍区域。
* 增加导航中状态保护，避免在已有 goal 执行过程中重复发送新目标。
* 增加基础日志输出，覆盖地图接收、frontier 选择、目标发送与导航结果等关键流程。
* 完成探索节点的基础工程结构，便于后续接入 Gazebo + SLAM Toolbox + Nav2 工作流。