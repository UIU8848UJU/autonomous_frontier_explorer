# exploration_learning 技术文档

## 1. 模块定位

`exploration_learning` 是探索学习数据管线包。当前只保留一条正式采集链路：

```text
DatasetRecorderNode
  -> EventBuffer
  -> DataRecordPluginFactory
  -> FrontierDecisionPlugin
  -> DatasetWriter
```

该包不负责训练模型、不负责 Gazebo/Nav2 多进程编排，也不修改在线 frontier 决策逻辑。

## 2. 当前目录结构

```text
exploration_learning/
├── include/exploration_learning/
│   ├── collector/                 # 采集底座、事件缓存、JSONL writer、plugin factory
│   ├── plugins/                   # record plugin 实现
│   ├── schema/                    # 训练 record schema 说明
│   ├── data_collection/           # episode 轻量生命周期工具
│   └── reward/                    # reward 计算工具
├── src/
│   ├── collector/
│   ├── plugins/
│   ├── data_collection/
│   ├── reward/
│   └── dataset_recorder_main.cpp
├── config/
│   └── dataset_recorder.yaml
├── launch/
│   ├── dataset_recorder.launch.py
│   └── rl_data_collection.launch.py   # 兼容入口，启动同一个 DatasetRecorderNode
└── test/
```

## 3. 组件职责

### DatasetRecorderNode

负责：

- 从 YAML 加载采集参数。
- 订阅 `/map`、frontier decision debug JSON、navigation result debug JSON 和 exploration state。
- 将 ROS 消息转换为统一 `TopicEvent`。
- 调用 `EventBuffer` 缓存上下文。
- 调用 `IDataRecordPlugin` 生成训练 record。
- 通过 `DatasetWriter` 写入 JSONL 和 episode metadata。

不负责：

- 训练模型。
- 批量启动仿真。
- reward 归因。
- 修改 frontier 在线策略。

### EventBuffer

按时间窗口缓存最近 topic 事件。插件可以用它查询最近 map summary、navigation result 和 exploration state。

### DataRecordPluginFactory

集中负责 plugin 实例化。节点不再用硬编码字符串 `if/else` 创建插件。当前已注册：

- `frontier_decision`

### FrontierDecisionPlugin

当前 MVP 在 frontier decision 事件到达时生成一条 `frontier_decision` record，并携带最近上下文：

- `frontier_context`
- `map_context`
- `outcome_context`
- `exploration_state_context`

上游仍是 debug JSON 字符串，因此第一版保留 raw JSON。后续可在该插件内部替换为正式 Candidate msg 或强 schema 解析。

### DatasetWriter

唯一保留的 writer 是 `exploration_learning::collector::DatasetWriter`。它只负责：

- 创建 episode 目录。
- 写 `episode_metadata.json`。
- append 写 `decision_records.jsonl`。

旧的占位 `exploration_learning::DatasetWriter` 已移除，避免同名不同职责。

## 4. 输出格式

输出路径：

```text
output_dir / episode_id / episode_metadata.json
output_dir / episode_id / decision_records.jsonl
```

`decision_records.jsonl` 每一行是独立 JSON object，当前字段包括：

- `record_type`
- `schema_version`
- `episode_id`
- `decision_id`
- `timestamp_sec`
- `selected_candidate_id`
- `candidates`
- `frontier_context`
- `map_context`
- `outcome_context`
- `exploration_state_context`
- `extra`

## 5. 参数

参数文件：

```text
config/dataset_recorder.yaml
```

关键参数：

- `output_dir`
- `episode_id`
- `episode_prefix`
- `writer_flush_every_n`
- `event_buffer_duration_sec`
- `plugin_name`
- `map_topic`
- `decision_topic`
- `navigation_result_topic`
- `exploration_state_topic`
- `record_map`
- `record_decision`
- `record_navigation_result`
- `record_exploration_state`

## 6. 构建、运行和测试

构建：

```bash
colcon build --symlink-install --packages-select exploration_learning
```

运行：

```bash
source install/setup.bash
ros2 launch exploration_learning dataset_recorder.launch.py
```

兼容入口：

```bash
ros2 launch exploration_learning rl_data_collection.launch.py
```

测试：

```bash
colcon test --packages-select exploration_learning --event-handlers console_direct+
colcon test-result --verbose
```

## 7. 后续开发建议

- 接入正式 Candidate / NavigationResult msg，替换 debug JSON。
- 在 `FrontierDecisionPlugin` 内解析候选列表，填充顶层 `candidates`。
- 加入 map gain / reward 归因，但保持与 recorder 解耦。
- 需要新增 record 类型时，只新增 plugin 并在 factory 注册，不扩展节点分支逻辑。
