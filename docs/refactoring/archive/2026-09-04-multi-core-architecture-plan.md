# 多核心架构重构与训练系统建设计划

> 归档说明：这是 2026-09-04 的设计基线，保留用于追溯架构决策。
>
> 当前实现参考：`d7be6c6`（2026-09-24）。文中的基线、缺口和工期判断不自动代表当前状态。

> 状态：PROPOSED（待项目负责人批准后冻结）
>
> 基线版本：`bd57629`（`main`）
>
> 编制日期：2026-09-04
>
> 估算口径：1 名熟悉 ROS 2 Humble、Nav2、C++17 的工程师；工作日为净工程投入，不含硬件排队、数据采集等待和外部依赖阻塞。

## 1. 执行摘要

本次演进分为两条相互解耦的主线：

1. **运行时架构重构**：把算法能力、ROS 通信、BT/TaskManager 编排和 Bringup 部署拆开。
2. **训练闭环建设**：以类型化事件采集决策数据，在 PC 端训练模型，以带版本和回退能力的模型制品回灌探索能力。

推荐目标架构：

```text
                         bringup
                            │
                       task_manager
                            │
             ┌──────────────┼──────────────┐
             ▼              ▼              ▼
      exploration_ros  navigation_ros   mapping_ros
             │              │              │
             ▼              ▼              ▼
      exploration_core navigation_core mapping_core
             └──────────────┼──────────────┘
                            ▼
                        core_types

      exploration_bt ──ROS contracts──► exploration_ros/navigation_ros
```

训练闭环：

```text
exploration_ros / navigation_ros / task_manager
                │
                ├── typed events ──► exploration_learning
                │                         │
                │                         ▼
                │                 versioned dataset
                │                         │
                │                         ▼
                │                    PC trainer
                │                         │
                │                         ▼
                ◄──── validated model + manifest
                       shadow → canary → active
```

总估算：

| 范围 | 净工作日 | 风险储备 | 单人日历预期 |
| --- | ---: | ---: | ---: |
| 架构重构主线（P0～P4、P6） | 38 | 约 8 | 约 9～10 周 |
| Learning 数据与模型接入（P5） | 8 | 约 2 | 约 2 周，另加数据采集等待 |
| **合计** | **46** | **约 9（20%）** | **约 11 周** |

如果只做第一阶段“`.so + node` 模板化”，10 个工作日即可交付一个行为兼容版本。两名工程师并行时建议按 7～8 周估算，不按人数简单减半，因为接口冻结、集成和仿真验收仍在关键路径上。

## 2. 当前仓库事实与修正结论

### 2.1 已确认现状

| 模块 | 当前事实 | 对计划的影响 |
| --- | --- | --- |
| `map_manager` | `map_manager_node` 把 node、`MapStorage` 一起编进单一 executable | 可先做链接目标拆分，但 `MapStorage` 当前持有 `rclcpp::Client<nav2_msgs::srv::SaveMap>`，不能直接称为纯 Core |
| `task_manager` | `TaskFlow` 已初步独立，但接口直接接收 `robot_interfaces::msg::ExplorationState` | 适合先提取 library；第二步用领域事件替换 ROS 消息依赖 |
| `frontier_explorer_core` | 已有 detector/pruner/scorer/selector/provider 分层和 GTest | 它仍直接依赖 `rclcpp`、`nav_msgs`、`nav2_msgs`、`geometry_msgs`、`tf2`，属于“算法库”，尚不是 ROS-free Core |
| `frontier_explorer_nodes` | 同一包内同时包含 frontier ROS wrapper、NavigationNode、BT plugin、BT orchestrator | 是第二阶段拆包的主要对象 |
| `exploration_learning` | 已有采集器、writer、reward/plugin 雏形和 2 个 GTest | 当前存在两套 recorder 路径，并使用 `std_msgs/String` JSON debug topic；应先收口 schema 和唯一采集链路 |
| `robot_interfaces` | 已有探索、导航、地图和任务消息/服务/action | 第一轮保持 wire contract 不变，新增学习事件时做向后兼容 |
| `autonomousr_explorer_bringup` | 已集中 launch/config/rviz | 名称有历史拼写，但不在架构重构中顺手改名，避免无关破坏 |

代码规模（仅 `.cpp/.hpp` 静态计数）：

| 包 | 文件数 | 行数 |
| --- | ---: | ---: |
| `frontier_explorer_core` | 39 | 3754 |
| `frontier_explorer_nodes` | 31 | 4218 |
| `exploration_learning` | 28 | 1479 |
| `map_manager` | 6 | 805 |
| `task_manager` | 7 | 615 |

当前工作环境未发现可用的 `colcon`/ROS 2 运行环境，也没有本地 `build/`、`install/` 证据，因此本文对现有行为的结论属于静态证据；P0 必须在 Ubuntu 22.04 + ROS 2 Humble 目标环境建立真实构建和仿真基线。

### 2.2 一个必须明确的架构区别

```text
生成 libxxx_core.so  ≠  Core 已经纯化
```

第一阶段只是把源文件从 executable 中提取成可链接库，主要解决构建目标和模板一致性。只有同时满足以下条件，才能宣布为“纯 Core”：

- 不包含或链接 `rclcpp`、ROS message、Nav2 action/service、TF2 ROS adapter；
- 不创建 topic/service/action/timer/client；
- 输入输出只使用 `core_types` 或 C++ 标准类型；
- 可以在没有 ROS graph 的进程中实例化和测试；
- 时间、日志、文件系统、模型推理等外部能力通过参数或窄接口注入。

## 3. 目标职责边界

### 3.1 Core = 能力

#### `core_types`

只放跨 Core 稳定、无 ROS 依赖的最小领域类型：

- `Pose2D`、`Point2D`、`GridMapView/GridMapSnapshot`；
- `FrontierCandidate`、`CandidateId`、score 分项；
- `NavigationRequest/Result/FailureReason`；
- `ExplorationEvent`、`TaskEvent`、状态枚举；
- schema/version/hash 等稳定标识。

禁止放入：节点参数读取、ROS message converter、业务算法、BT blackboard、万能工具函数。`core_types` 不应演变为“公共垃圾桶”。

#### `exploration_core`

- frontier detection、pruning、scoring、selection；
- retry/blacklist 策略状态；
- rule ranker 与可插拔 learned ranker 接口；
- 不订阅地图、不查 TF、不发布 marker、不调用 Nav2。

#### `navigation_core`

- footprint collision、path safety、代价阈值判断；
- 导航结果归一化与可重试性分类；
- 不创建 Nav2 action client，不读取 costmap topic。

#### `mapping_core`

- 地图统计、unknown ratio、完成判定窗口；
- 保存请求的领域状态和文件命名策略；
- 不调用 `/map_saver/save_map`，不持有 ROS node/client。

### 3.2 ROS = 通信和适配

#### `exploration_ros`

- ROS 参数到 Core 配置转换；
- `/map`、costmap、TF 输入适配；
- frontier services、state、marker、学习事件发布；
- `core_types ↔ robot_interfaces` 转换；
- 托管 `exploration_core` 生命周期。

#### `navigation_ros`

- 对外导航 action/feasibility service；
- 对内桥接 Nav2 `ComputePathToPose`、`NavigateToPose` 和 costmap topic；
- 把 ROS/Nav2 错误转换为 `navigation_core` 结果。

#### `mapping_ros`

- `/map`、探索状态、最终地图和 map manager state；
- Nav2 SaveMap service adapter；
- 托管 `mapping_core`。

### 3.3 BT / TaskManager = 编排

#### `exploration_bt`

- BT XML、插件、blackboard context、tick 生命周期；
- 编排 Detect/Score/Select/Navigate/Recovery；
- 通过稳定 ROS contract 调用能力，不直接访问 Core 内部状态；
- retry/blacklist 的策略数据仍归 `exploration_core`，BT 只发送“导航失败”等事实事件。

#### `task_manager`

- 管理 Mapping/Exploration/Navigation 顶层任务状态；
- 决定何时启动、停止、暂停和恢复能力；
- 不实现 frontier 算法、路径安全或地图统计。

### 3.4 Bringup = 部署

- 只包含 launch、YAML、RViz 和 deployment profile；
- 不包含业务状态机和算法；
- 对外只选择“启动哪些组件、命名空间、参数文件、是否仿真”；
- launch 不复制包内默认参数，参数真源唯一。

### 3.5 Learning = 优化能力

- 采集、关联、校验、落盘数据；
- PC 端训练、评估和制品生成；
- runtime 只负责推理与回退，不在机器人端在线训练；
- 硬安全过滤永远先于 ML 排序；模型不可用时自动回退 rule ranker。

## 4. 重构不可破坏项

第一阶段到 P4 结束前，以下项目默认冻结：

1. executable 名和 launch 对外入口；
2. topic/service/action 名、消息字段及 QoS；
3. YAML 参数名、默认值和覆盖优先级；
4. frontier 候选排序、retry/blacklist、完成判定和导航失败语义；
5. state code、状态迁移和错误文本的机器可观察部分；
6. 地图文件命名与保存触发时机；
7. 仿真地图、机器人模型、Nav2/SLAM 参数。

确需改变以上任一项时，必须单独提交 Change Request，不得夹在“架构重构”提交中。

## 5. 分阶段实施与工作日

### P0：行为基线与设计冻结（D1～D3，3 日）

| 工作日 | 工作内容 | 交付物 |
| --- | --- | --- |
| D1 | 在目标 Ubuntu/ROS 环境完整构建；记录工具链、依赖、命令、耗时和产物 | `baseline/build.md`、原始日志、包依赖图 |
| D2 | 建立最小仿真 golden run；记录接口列表、参数 dump、状态序列、关键 topic bag | `baseline/runtime.md`、rosbag、golden trace |
| D3 | 冻结边界、目标包图、兼容清单和 PR 切片；建立基线标签 | 设计评审记录、接口清单、回滚 tag |

退出门禁 G0：

- 全仓构建和现有测试结果已记录；
- 至少 1 次从 `/start_mapping` 到探索运行/停止的可复现仿真；
- 所有公共接口和参数已清点；
- 如果基线本身失败，失败必须登记，后续只允许保持“不比基线更差”，不得伪称重构导致。

### P1：`map_manager` / `task_manager` 链接目标拆分（D4～D10，7 日）

这是风险最低、行为不变的模板阶段。

| 工作日 | 工作内容 | 交付物 |
| --- | --- | --- |
| D4 | 为 `MapStorage`、地图统计/完成判定补 characterization test 与 clock/filesystem seam | RED 测试、现有行为样本 |
| D5 | 提取 `map_manager_core` library，node executable 只保留 `main + node` | `libmap_manager_core.so`、原名 `map_manager_node` |
| D6 | 验证安装/export/include/package 依赖；运行 map manager 测试 | 构建和测试日志 |
| D7 | 为 `TaskFlow` 状态迁移、重复命令、超时/失败补 characterization test | 状态迁移测试矩阵 |
| D8 | 提取 `task_manager_core` library，node executable 只保留 `main + node` | `libtask_manager_core.so`、原名 `task_manager_node` |
| D9 | 保持原 launch、服务和参数不变，完成包级 smoke test | 接口 diff = 0 |
| D10 | 全系统仿真回归、审查模板、冻结 P1 | P1 验收报告和可复制模板 |

P1 目标结构：

```text
libmap_manager_core.so        libtask_manager_core.so
          ▲                              ▲
          │                              │
 map_manager_node                 task_manager_node
   main + ROS wrapper              main + ROS wrapper
```

退出门禁 G1：

- 公共 executable、topic/service/action、参数完全不变；
- 新增 library 正确安装并导出；
- Characterization test 全绿；
- golden trace 的状态迁移与重构前一致；
- 明确标注 `map_manager_core` 此时可能仍含 ROS adapter，只完成链接拆分，不宣称纯 Core。

### P2：`core_types` 与 Core 去 ROS 化（D11～D18，8 日）

| 工作日 | 工作内容 | 交付物 |
| --- | --- | --- |
| D11 | 设计最小领域类型、所有权、错误枚举和 converter 位置 | `core_types` API 草案 |
| D12 | 建立 `core_types` 包和无 ROS 单测；加入禁止依赖检查 | 可安装的 headers/library |
| D13 | `TaskFlow` 改为接收 `TaskEvent/ExplorationEvent`，ROS msg 转换移到 node | 纯 `task_manager_core` |
| D14 | 从 map node 提取地图统计、unknown 稳定窗口和完成判定 | `mapping_core` 第一版 |
| D15 | 把 SaveMap client 留在 `mapping_ros` adapter；给 clock/filesystem 注入 seam | 纯 `mapping_core` |
| D16 | 将 frontier 类型和地图视图逐步替换为 `core_types` | ROS-free 算法输入输出 |
| D17 | 清理 `rclcpp::Logger`、ROS 时间和 ROS geometry 类型；由 wrapper 转换 | 纯 `exploration_core` |
| D18 | 依赖审计、核心单测、行为 replay、G2 评审 | Core purity 报告 |

退出门禁 G2：

- `core_types` 不依赖任何 ROS/Nav2 包；
- `exploration_core`、`mapping_core`、`task_manager_core` 的公共头文件中不出现 ROS/Nav2 类型；
- Core 内没有 node/client/publisher/subscription/timer；
- Core 测试无需启动 ROS graph；
- ROS converter 只存在于 `*_ros` 或 adapter 目录；
- golden 输入下候选排序、完成判断和任务状态迁移保持一致。

### P3：拆分 `frontier_explorer_nodes`（D19～D28，10 日）

| 工作日 | 工作内容 | 交付物 |
| --- | --- | --- |
| D19 | 冻结包拆分图和循环依赖检查；先调整 install/export | 可构建迁移骨架 |
| D20～D21 | 迁移 `FrontierExplorerNode`、TF/map adapter、marker 到 `exploration_ros` | `exploration_ros` 包 |
| D22～D23 | 提取 path/footprint/result policy 到 `navigation_core` | `navigation_core` + 单测 |
| D24～D25 | 迁移 NavigationNode、Nav2 action/service adapter 到 `navigation_ros` | `navigation_ros` 包 |
| D26～D27 | 迁移 BT XML、plugins、context、orchestrator 到 `exploration_bt` | `exploration_bt` 包 |
| D28 | 兼容 launch、接口 diff、全系统仿真与 G3 评审 | P3 验收报告 |

退出门禁 G3：

- 原 `frontier_explorer_nodes` 不再同时承载探索、导航和 BT 三类职责；
- 依赖方向满足 `BT/ROS → Core → core_types`，不存在 Core 反向依赖 ROS/BT；
- BT 只使用公开 contract，不 include Core 内部 state；
- NavigationNode 的单目标保护、cancel、feedback、feasibility/path safety 语义不变；
- 旧 launch 对外入口至少保留一个版本的兼容转发，并输出 deprecation 日志。

### P4：Bringup、接口和部署收口（D29～D33，5 日）

| 工作日 | 工作内容 | 交付物 |
| --- | --- | --- |
| D29 | 更新 `robot_interfaces` 依赖归属，清理包间循环和多余依赖 | 依赖图、manifest diff |
| D30 | 更新 bringup launch/config，保留旧参数兼容映射 | 新旧 launch smoke test |
| D31 | 修复 `build.sh` 和 README 中已过期的 `frontier_explorer` 包名/路径 | 可用开发命令和文档 |
| D32 | 验证 clean workspace 构建、安装后运行和 selective build | clean-build 证据 |
| D33 | 回滚演练：切回兼容入口/旧模型/规则 ranker | 发布与回滚 runbook |

退出门禁 G4：

- 干净 workspace 可以按文档一次构建；
- `ros2 launch autonomousr_explorer_bringup full_system.launch.py` 或批准的新入口可启动；
- 安装空间中所有 library、plugin XML、BT XML、config 均能被发现；
- launch 参数有唯一真源，无静默复制和漂移；
- 完成一次可操作的回滚演练。

### P5：Learning 数据—训练—模型闭环（D34～D41，8 日净投入）

数据积累需要额外日历时间，不计入 8 个工程工作日。

| 工作日 | 工作内容 | 交付物 |
| --- | --- | --- |
| D34 | 定义 `decision_id/episode_id/map_id`、feature/label/event schema 和版本策略 | schema v1、数据字典 |
| D35 | 新增类型化 Decision/NavigationOutcome/Episode 事件；保留 String debug 兼容 | ROS event contract |
| D36 | 收口重复 recorder/writer，只保留唯一生产链路 | 单一 collector pipeline |
| D37 | 实现事件关联、丢失/重复检测、原子落盘和 dataset manifest | versioned dataset |
| D38 | PC trainer 最小流水线：split、训练、指标、可复现 seed | trainer CLI + report |
| D39 | 定义模型制品 `model + manifest + feature_schema + metrics + hash`；做 runtime spike | 可验证模型包 |
| D40 | 接入 shadow mode，规则继续执行，模型只记录排名差异 | shadow comparison report |
| D41 | canary 开关、健康检查、加载失败回退和一键禁用 | deployment policy + G5 |

建议的数据契约：

```text
DecisionEvent
  schema_version
  episode_id / decision_id / timestamp
  map_id / map_revision / robot_pose
  candidates[]
    candidate_id / geometry
    rule_score + score_components
    safety_features / path_features
  rule_selected_candidate_id
  model_selected_candidate_id (optional)

NavigationOutcomeEvent
  decision_id / candidate_id
  accepted / succeeded / canceled / failure_reason
  path_length / travel_time / recovery_count
  observed_information_gain
```

模型制品最低要求：

- 明确 `model_version`、`schema_version`、训练代码 commit、数据集 hash；
- 声明 feature 顺序、归一化、缺失值行为和支持的 runtime；
- 加载前校验 hash 和 schema compatibility；
- 模型只对通过硬安全过滤的候选排序；
- 超时、NaN、维度不匹配、模型缺失时立即回退规则排序；
- 默认先 shadow，再 canary，最后 active；支持参数一键退回 `rule_only`。

推理格式优先评估 ONNX，但在完成 ARM/x86、延迟、包体和部署 spike 前不把 ONNX Runtime 写成不可变依赖。

退出门禁 G5：

- 数据记录完整率 ≥ 99%，无法关联 outcome 的 decision < 1%，损坏记录为 0；
- 同一数据集、代码 commit 和 seed 可复现指标；
- shadow 模式不影响控制链路时序和安全过滤；
- 模型不可用/超时时 100% 回退 rule ranker；
- canary 只有在离线指标和仿真指标均过线后才能启用。

### P6：系统验收与发布（D42～D46，5 日）

| 工作日 | 工作内容 | 交付物 |
| --- | --- | --- |
| D42 | 全量 unit/component/contract test | 测试报告 |
| D43 | SLAM 探索、静态地图导航、stop/cancel/restart 故障注入 | 集成测试报告 |
| D44 | 性能、长稳、重复启动和资源释放测试 | 性能/稳定性报告 |
| D45 | 文档、架构图、接口手册、排障和回滚手册审查 | 发布文档包 |
| D46 | 最终验收、tag、发布说明和下一阶段 backlog | release candidate |

退出门禁 G6：

- 所有 Must 级验收要求通过；
- 没有 Critical/Major 未知项；
- 发布包、配置、模型和文档均有版本；
- 独立审查通过；
- 可以从上一稳定版本回滚且数据集仍可读取。

## 6. 验收要求

### 6.1 Must（发布阻断）

| ID | 要求 | 验证方法 |
| --- | --- | --- |
| AR-001 | 依赖方向只允许 `bringup/task/BT/ROS → Core → core_types` | 自动生成包图 + include/link 审计 |
| AR-002 | `core_types` 和三个能力 Core 的 public API 不含 ROS/Nav2 类型 | `rg`/编译隔离检查 + standalone test |
| AR-003 | P1～P4 不改变既有公共 ROS contract | `ros2 interface`、node info、参数 dump 前后 diff |
| AR-004 | 同一 golden 输入产生相同候选顺序、状态迁移和保存触发 | deterministic replay / characterization tests |
| AR-005 | 干净环境全仓 build/test 通过 | clean `colcon build` + `colcon test-result --verbose` |
| AR-006 | Core 关键状态迁移和失败分支 100% 场景覆盖 | 测试矩阵审查；不以单一行覆盖率替代 |
| AR-007 | 新增/修改 Core 代码行覆盖率 ≥ 80%，关键分支 ≥ 90% | 覆盖率报告 |
| AR-008 | 10 次连续启动/停止无崩溃、悬挂 goal 或残留进程 | launch repeat test |
| AR-009 | p95 决策耗时不比 P0 基线回退超过 5%，RSS 不回退超过 10% | 相同地图、配置和硬件对比 |
| AR-010 | 任一新包/模型可通过配置回退至已验证路径 | 回滚演练 |
| AR-011 | ML 不得绕过 collision/unknown/path/blacklist 硬安全规则 | 安全顺序单测 + 故障注入 |
| AR-012 | 模型加载失败、schema 不匹配或推理超时自动 rule fallback | component/integration test |

### 6.2 Should（不阻断首版，但需建账）

- 修改某个 Core 时，不触发无关 ROS/BT 包重编译；以 P0 clean/incremental build 数据评估改善。
- Core 单测可在普通 C++ 测试进程执行，单轮目标小于 10 秒。
- 每个 package 只有一个清晰的职责说明、owner 和 public API 列表。
- 所有 deprecated launch/service/parameter 给出移除版本，不无限期兼容。
- 学习模型相对 rule baseline 的目标：导航成功率提升 ≥ 5% 或单位时间 information gain 提升 ≥ 5%，同时安全失败率不得上升。未达到时保留 shadow，不强行上线。

## 7. 预期重构效果

| 维度 | 重构前 | 完成后的目标效果 |
| --- | --- | --- |
| 职责 | 单包同时包含算法、ROS、导航和 BT | 每层有单一职责和单向依赖 |
| 测试 | 大量逻辑需连 ROS 类型/环境测试 | Core 可快速、确定性、无 ROS graph 单测 |
| 构建 | executable 直接编入多类源文件 | 稳定 library + 薄 node，增量构建边界清楚 |
| 复用 | 算法与 ROS/Nav2 数据类型绑定 | Core 可用于离线 replay、训练特征生成和其他 runtime |
| 变更风险 | 改策略可能影响通信/编排 | 能力、适配、编排、部署可独立变更和回滚 |
| 可观测性 | String JSON debug 与状态 topic 混用 | 类型化事件、关联 ID、schema 和 dataset manifest |
| ML 接入 | 模型接口与数据闭环未冻结 | rule/shadow/canary/active 有明确晋级和回退机制 |
| 部署 | launch 与包路径存在历史漂移 | Bringup 是唯一部署入口，安装后可发现全部资源 |

不能把“文件数增加”当作成功。真正效果以依赖审计、无 ROS Core 测试、行为等价、构建影响范围和回滚能力衡量。

## 8. 测试策略

| 层级 | 测试对象 | 重点 |
| --- | --- | --- |
| Unit | detector/pruner/scorer/selector、map completion、TaskFlow、path safety | 正常、边界、非法输入、超时分类、重试、确定性 |
| Converter contract | `core_types ↔ ROS msg` | 字段完整、枚举映射、单位/frame/time 语义 |
| Component | 每个 `*_ros` node | 参数、QoS、service/action、生命周期、错误传播 |
| BT | 每个 plugin + XML | SUCCESS/FAILURE/RUNNING、halt/cancel、恢复和重新选点 |
| Launch smoke | bringup | 资源发现、命名空间、参数加载、启动/停止 |
| System | Gazebo + SLAM + Nav2 | 建图探索、完成保存、静态导航、故障恢复 |
| Replay | rosbag/golden dataset | 重构前后候选和状态差异 |
| Learning | schema、join、writer、trainer、inference fallback | 丢失、重复、乱序、损坏、版本不匹配、超时 |

最低故障注入清单：

- map_saver 不可用、返回 false、超时；
- Nav2 goal reject、abort、cancel 超时；
- TF 暂时不可用；
- map/costmap 未到、尺寸变化、空数据；
- BT plugin/BT XML 不存在或加载失败；
- 重复 start/stop、并发请求、重启过程中再次启动；
- dataset 目录不可写、磁盘满、半条记录；
- 模型文件缺失、hash 错误、schema 不兼容、输出 NaN、推理超时。

## 9. PR 与分支切片

禁止用一个大 PR 同时完成全部拆分。推荐切片：

1. `PR-01 baseline-and-contract-tests`
2. `PR-02 map-manager-library-extraction`
3. `PR-03 task-manager-library-extraction`
4. `PR-04 core-types-and-converters`
5. `PR-05 mapping-core-purification`
6. `PR-06 exploration-core-purification`
7. `PR-07 exploration-ros-package`
8. `PR-08 navigation-core-and-ros-packages`
9. `PR-09 exploration-bt-package`
10. `PR-10 bringup-compatibility-and-docs`
11. `PR-11 typed-learning-events-and-dataset-v1`
12. `PR-12 model-shadow-and-fallback`
13. `PR-13 final-system-acceptance`

每个 PR 必须满足：自身可构建、测试可复现、公共 contract diff 已说明、没有同时混入业务策略调参。

## 10. 风险与应对

| 风险 | 概率 | 影响 | 应对 |
| --- | --- | --- | --- |
| 抽类型时单位/frame/time 语义丢失 | 中 | 高 | converter contract test；类型名显式携带 frame/单位语义 |
| 把 ROS adapter 错放进 Core | 高 | 中 | public include 审计 + 禁止依赖列表 |
| 拆包后 plugin/resource 安装路径错误 | 中 | 高 | install-space launch test，不只测源码空间 |
| BT halt 与 Nav2 cancel 竞态 | 中 | 高 | Characterization + 并发/重复 stop 测试 |
| map save async callback 生命周期问题 | 中 | 高 | ownership 测试、节点销毁和服务超时注入 |
| 兼容入口长期保留形成双实现 | 中 | 中 | 兼容层只做转发，声明移除版本 |
| 学习事件用 String JSON 继续漂移 | 高 | 中 | typed event + schema version；String 仅 debug |
| dataset label 关联错误 | 中 | 高 | `episode_id + decision_id + candidate_id`，完整性审计 |
| 模型提升离线指标但降低实机安全/稳定 | 中 | 高 | 硬安全前置、shadow、canary、自动回退 |
| 当前 Windows 工作区无法验证 ROS runtime | 高 | 中 | 所有 Gate 在目标 Ubuntu/ROS 环境执行并保存原始日志 |

## 11. 回滚策略

- 每个 Gate 建立可运行 tag，不跨多个 Gate 才首次集成。
- P1 保持原 executable 名；回滚只需恢复旧 link/source 列表。
- P3 旧 launch 保留兼容转发一个版本；新包失败时切回旧入口。
- ROS contract 采用 additive change，禁止原地改变已发布字段语义。
- Learning 默认 `rule_only`；`shadow/canary/active` 由单一参数控制。
- 模型目录保留 last-known-good manifest；新模型健康检查失败不替换活动软链接/配置。
- 数据 schema 只新增版本，不覆盖历史数据；trainer 明确声明支持版本。

## 12. Definition of Done

整个计划只有在以下条件全部满足后才能关闭：

- 目标包图与实际 `package.xml/CMakeLists.txt/include` 依赖一致；
- Core 纯度、接口兼容、行为 replay、性能和稳定性达到 Must 要求；
- 建图探索、地图保存、静态导航、停止/恢复均有系统证据；
- 文档命令在 clean workspace 实际执行过；
- Learning 数据可追溯，模型可验证、可灰度、可回退；
- 没有靠 README 声明代替自动检查；
- 独立 Reviewer 通过最终门禁；
- 发布和回滚手册由非作者按步骤演练成功。

## 13. 建议的近期执行顺序

近期不要直接创建所有目标包。先按下面顺序启动：

```text
P0 baseline
  ↓
map_manager characterization
  ↓
libmap_manager_core.so + unchanged node
  ↓
task_manager characterization
  ↓
libtask_manager_core.so + unchanged node
  ↓
full-system regression
  ↓
再批准 core_types / ROS-free Core 设计
```

这样前 10 个工作日就能得到一个稳定模板和明确收益；若 P1 发现现有行为不稳定，也可以停在测试基线阶段，不会把风险扩散到 exploration、navigation、BT 和 Learning。
