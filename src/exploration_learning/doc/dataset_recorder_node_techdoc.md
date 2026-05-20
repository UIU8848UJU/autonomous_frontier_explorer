# DatasetRecorderNode 技术说明

## 1. 模块定位

`DatasetRecorderNode` 属于 `exploration_learning` 包，只负责 ROS 内部数据采集和 jsonl 落盘。

它不负责批量启动 Gazebo、Nav2、SLAM 或 frontier exploration。批量仿真 episode 的进程编排由工作空间根目录的 `scripts/run_dataset_collection.py` 负责。

第一版目标是建立稳定的数据采集入口，不引入 ML 模型、不实现训练逻辑，也不修改 frontier 决策链路。

## 2. 数据流

当前数据流如下：

```text
/map
  -> DatasetRecorderNode
  -> record_type: map_summary

/frontier_explorer/decision_debug_json
  -> DatasetRecorderNode
  -> record_type: decision_debug_json

/navigation/navigation_result_debug_json
  -> DatasetRecorderNode
  -> record_type: navigation_result_debug_json
```

`/map` 使用 `nav_msgs/msg/OccupancyGrid`，节点只统计摘要，不写入完整 `data` 数组。

`/frontier_explorer/decision_debug_json` 和 `/navigation/navigation_result_debug_json` 暂时使用 `std_msgs/msg/String`，用于预留上游调试 JSON 接口。后续如果 `robot_interfaces` 中新增了正式消息，可以替换这两个临时 topic。

## 3. 输出格式

输出路径：

```text
dataset_output_dir / episode_id / records.jsonl
```

每一行是一个独立 JSON object，基础字段包括：

- `timestamp`：记录时间戳，单位秒。
- `episode_id`：当前 episode 标识。
- `record_type`：记录类型。
- `payload`：不同类型记录的载荷。

`map_summary` 的 payload 包括：

- `width`
- `height`
- `resolution`
- `known_cells`
- `unknown_cells`
- `occupied_cells`
- `free_cells`

episode metadata 已通过 `EpisodeMetadata` 结构体预留，未来可扩展为单独的 `metadata.json` 或每个 episode 的首行 metadata record。

## 4. 为什么不让 node 拉起 Gazebo

Gazebo、Nav2、SLAM、RViz 和探索节点属于多个独立进程。批量采集时通常还需要处理：

- 进程启动顺序。
- 超时控制。
- 异常退出和 kill。
- 多 episode 循环。
- 并发采集。
- `ROS_DOMAIN_ID` 隔离。
- 仿真世界、随机种子和初始位姿切换。

这些工作更适合放在外部 Python 编排脚本中，而不是写进 ROS node。`DatasetRecorderNode` 保持单一职责，只做数据接收和文件写入。

## 5. 后续扩展计划

- 接入真实 Candidate msg，替换临时 `decision_debug_json` 字符串 topic。
- 记录 selected candidate、未选中候选和 reject reason。
- 记录 Nav2 action result、路径长度、失败原因和恢复状态。
- 根据连续 map summary 计算 `map_gain`。
- 支持按 episode 同步写 rosbag2。
- 支持导出 ML 训练所需的 JSONL、CSV、Parquet 或自定义 tensor 数据格式。
