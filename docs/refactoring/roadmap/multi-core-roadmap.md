# 多核心架构

> 状态：持续路线图
>
> 同步基线：`d7be6c6`（2026-09-24）
>
> 说明：核心与 ROS 适配层的主要拆分已经落地；地图 I/O、多机器人和训练闭环仍按需推进。

```text
                    bringup / task_manager
                           │
           ┌───────────────┼───────────────┐
           ▼               ▼               ▼
    exploration_ros   navigation_ros   mapping_ros
           │               │               │
           ▼               ▼               ▼
    exploration_core  navigation_core  mapping_core
           │               │
           └───────┬───────┘
                   ▼
              core_types
```

# 训练系统
```text
exploration_ros
      │
      ├──── topic/event ───► exploration_learning
      │                            │
      │                            ▼
      │                         dataset
      │                            │
      │                            ▼
      │                       PC trainer
      │                            │
      ◄──────── model ─────────────┘
```

# 阶段计划


map_manager
task_manager

core.cpp + node.cpp
        ↓
一个 executable

改成：

map_manager_core.so
        ↑
map_manager_node

task_manager_core.so
        ↑
task_manager_node

这样属于纯架构重构，不改业务逻辑，风险最低，也能先建立标准模板。

# 第二阶段
frontier_explorer_nodes

重新整理成：

exploration_core       已有
exploration_ros
exploration_bt

这里 BT 我倾向于单独看成编排能力，而不是 Core：

BT:
Detect
  ↓
Score
  ↓
Select
  ↓
Navigate
  ↓
Recovery

它负责“能力怎么组合”，不是算法本身。

# 最后
Core = 能力
ROS = 通信
BT/TaskManager = 编排
Bringup = 部署
Learning = 优化能力

# 地图生命周期与 I/O 后续规划

当前阶段保留单一的生命周期节点形态：

```text
SLAM / map
    ↓
map_lifecycle_node
    ├── MapManagerCore
    └── Nav2MapSaver（当前保存 Adapter）
```

当前 `map_manager` 包名、`map_manager_state` 状态接口和已有 topic 保持兼容；节点内部命名统一使用 `MapLifecycleNode` / `map_lifecycle_node`。当前保存继续通过 Nav2 的 map_saver 服务完成，暂不单独创建 `map_io_node` 进程，也不在生命周期核心中引入文件格式和 ROS/Nav2 细节。未来如需运行时加载，再增加 Nav2 map_server 的读取 Adapter。

静态地图模式保持简单链路：

```text
map_server / map_io
        ↓
Navigation
```

静态模式不需要参与探索完成判断和自动保存的 lifecycle 节点。

只有出现多机器人、多读多写、地图版本协同、写入仲裁、锁/租约、冲突合并或远程地图仓库时，再规划独立的 `map_pack`：

```text
map_pack
├── map_lifecycle       # 地图版本和生命周期编排
├── map_io              # 读写协议与后端适配
└── map_storage         # 本地、共享盘或远程地图存储
```

届时将 `map_io_core` 设计为纯 C++ 能力层，由 ROS、DDS、文件系统或远程存储 Adapter 接入；2D OccupancyGrid、3D VoxelMap、PointCloud 和 Traversability Map 通过不同地图后端扩展。单机器人单写入阶段继续复用 Nav2 的 saver/server，避免提前引入进程间同步和一致性复杂度。
