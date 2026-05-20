# exploration_learning 技术文档

## 1. 模块定位

`exploration_learning` 是探索学习数据管线的预留包，当前版本先完成可编译架构骨架。它不直接参与 `frontier_explorer_nodes` 的在线决策，而是作为后续强化学习、模仿学习或离线评估的数据采集入口。

模块目标：

- 从探索流程中收集状态、动作、导航结果和地图变化。
- 将一次探索过程切分为 episode 和 transition。
- 根据探索收益、路径代价和失败事件计算 reward。
- 将样本写入稳定的数据集目录，供离线训练或分析使用。

## 2. 当前目录结构

```text
exploration_learning/
├── include/exploration_learning/
│   ├── data_collection/
│   │   ├── episode_recorder.hpp
│   │   └── rl_data_collector_node.hpp
│   ├── dataset/
│   │   └── dataset_writer.hpp
│   └── reward/
│       └── reward_calculator.hpp
├── src/
│   ├── dataset_writer.cpp
│   ├── episode_recorder.cpp
│   ├── reward_calculator.cpp
│   ├── rl_data_collector_main.cpp
│   └── rl_data_collector_node.cpp
├── config/
│   └── rl_data_collector.yaml
└── launch/
    └── rl_data_collection.launch.py
```

## 3. 组件职责

### RlDataCollectorNode

`RlDataCollectorNode` 是 ROS 2 节点入口，当前负责声明和读取参数，并初始化内部组件。

后续建议接入的输入：

- `/frontier_explorer/state`：frontier 能力节点状态。
- `/exploration_orchestrator/state`：BT 编排状态。
- `/map`：用于计算 explored area delta。
- frontier candidate / selected goal 服务结果：用于记录决策输入和动作。
- navigation result：用于记录 reachability、feasibility、goal reached、failure reason。

### EpisodeRecorder

`EpisodeRecorder` 负责 episode 生命周期和 transition 计数。当前字段包括：

- `active_`：是否正在记录 episode。
- `episode_id_`：当前 episode 标识。
- `transition_count_`：当前 episode 已记录 transition 数量。

后续可扩展为：

- 记录 episode 起止时间。
- 记录初始地图、最终地图和总探索面积。
- 缓存 transition，批量交给 `DatasetWriter`。

### RewardCalculator

`RewardCalculator` 将 `RewardInput` 转换为标量 reward。当前 reward 形式为：

```text
reward = explored_area_delta - 0.05 * path_length_delta
```

并额外处理：

- `reached_goal == true` 时增加 goal bonus。
- `collision == true` 时施加较大惩罚。

该类保持无状态，便于单元测试和离线复用。

### DatasetWriter

`DatasetWriter` 当前只保存 dataset 输出路径。后续应负责：

- 创建 episode 目录。
- 写入 transition 行数据。
- 写入 metadata，例如地图名、参数版本、机器人类型、采集时间。
- 支持 JSONL、CSV、Parquet 或 rosbag2 派生格式。

## 4. 推荐数据流

```text
ROS topics/services/actions
        |
        v
RlDataCollectorNode
        |
        +--> EpisodeRecorder: 管理 episode 与 transition 边界
        |
        +--> RewardCalculator: 根据地图增量和导航结果计算 reward
        |
        +--> DatasetWriter: 写入样本与元数据
```

一次 transition 推荐包含：

- observation：机器人位姿、局部地图统计、frontier candidate 特征、黑名单状态。
- action：选中的 frontier goal 或候选编号。
- reward：由 `RewardCalculator` 输出。
- next_observation：动作执行后的新状态。
- done：episode 是否结束。
- info：失败原因、BT 状态、Nav2 result code、参数快照。

## 5. 参数

当前参数文件为 `config/rl_data_collector.yaml`：

```yaml
rl_data_collector_node:
  ros__parameters:
    dataset_path: exploration_dataset
```

`dataset_path` 表示未来样本输出目录或数据集名称。后续可增加：

- `episode_timeout_sec`
- `record_map_snapshots`
- `record_frontier_candidates`
- `dataset_format`
- `flush_every_n_transitions`

## 6. 构建与运行

构建：

```bash
colcon build --symlink-install --packages-select exploration_learning
```

运行：

```bash
ros2 launch exploration_learning rl_data_collection.launch.py
```

## 7. 后续开发建议

- 先定义 transition schema，再实现 `DatasetWriter`，避免采集格式频繁变化。
- 将 ROS message 到学习特征的转换放在 node 或 adapter 中，保持 `RewardCalculator` 可测试。
- 将 reward 权重参数化，不要把训练实验参数硬编码在算法类中。
- 为 `RewardCalculator` 和 dataset schema 增加单元测试。
- 在 full system launch 中以可选开关启动 `rl_data_collector_node`，避免默认运行影响探索链路。
