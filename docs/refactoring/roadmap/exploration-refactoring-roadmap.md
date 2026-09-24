# 目标

> 状态：部分完成，保留为后续训练闭环路线图
>
> 同步基线：`d7be6c6`（2026-09-24）
>
> 已落地：探索行为与 Frontier 策略分离、ROS-free Core、Ranker 接缝、统一地图/机器人几何、真实信息增益与候选连续选择能力。
>
> 待继续：训练数据闭环、模型版本管理、离线评估及学习策略灰度接入。

本轮不继续扩大功能，目标只有三个：

1. 将“探索”与“Frontier 算法”分离。
2. 将 Navigation 中的机器人几何能力抽离，并让 Navigation Core ROS-free。
3. 打通 `Observation → Decision → Outcome` 数据闭环，达到第一版机器学习训练条件。

要求：

* 现有 ROS topic、action、service、launch 行为尽量保持兼容。
* 不做无关重命名和大规模目录搬迁。
* Core 禁止依赖 ROS/Nav2。
* 新 Core 必须有单元测试和 ROS 依赖边界测试。
* 第一阶段优先保证行为不变，再做名称迁移。

---

# Phase 1：建立通用 Exploration Policy

新增纯 C++ `exploration_core`。

目标结构：

```text
exploration_core/
├── types/
│   ├── exploration_observation.hpp
│   ├── exploration_decision.hpp
│   ├── exploration_outcome.hpp
│   └── exploration_status.hpp
└── policy/
    └── exploration_policy.hpp
```

核心接口：

```cpp
class IExplorationPolicy
{
public:
    virtual ~IExplorationPolicy() = default;

    virtual void reset() = 0;

    virtual ExplorationDecision decide(
        const ExplorationObservation & observation) = 0;

    virtual void on_outcome(
        const ExplorationOutcome & outcome) = 0;
};
```

职责：

```text
Explore
= 完成自主探索这个任务

Frontier
= Explore 的一种策略

未来 ML / RL
= Explore 的其他策略
```

最终关系：

```text
Exploration Executor / BT
        ↓
IExplorationPolicy
        │
        ├── FrontierExplorationPolicy
        ├── LearnedExplorationPolicy   # future
        └── RLExplorationPolicy        # future
```

本阶段不要删除或大规模重命名现有 `frontier_explorer_*` package。

先建立正确依赖方向。

---

# Phase 2：Frontier 降级为 Exploration 的一种算法

保留现有：

```text
FrontierDetector
FrontierPruner
FrontierScorer
FrontierSelectionPolicy
```

新增：

```text
FrontierExplorationPolicy
```

实现：

```cpp
class FrontierExplorationPolicy : public IExplorationPolicy
{
    ...
};
```

内部组合：

```text
GridMap
  ↓
FrontierDetector
  ↓
FrontierPruner
  ↓
Candidate Ranker
  ↓
Selection
  ↓
ExplorationDecision
```

重点：不要让 `Exploration` Core 知道什么是 Frontier。

依赖方向必须是：

```text
exploration_core
        ↑
frontier_explorer_core
```

禁止：

```text
exploration_core
        ↓
frontier_explorer_core
```

---

# Phase 3：给机器学习预留真正的替换点

第一版机器学习不要直接替换整个 Exploration Policy。

先学习：

```text
Frontier Candidate Ranking
```

新增：

```cpp
class IFrontierRanker
{
public:
    virtual std::vector<ScoredFrontierCandidate> rank(
        const std::vector<FrontierCandidate> & candidates) = 0;
};
```

现有规则算法包装为：

```text
RuleBasedFrontierRanker
```

未来：

```text
LearnedFrontierRanker
```

结构：

```text
FrontierExplorationPolicy
├── Detector
├── Pruner
├── IFrontierRanker
│   ├── RuleBasedFrontierRanker
│   └── LearnedFrontierRanker
└── Selector
```

第一版训练因此只需要学习：

```text
candidate features
        ↓
expected utility / ranking score
        ↓
选择最优 candidate
```

而不是让神经网络直接从整张地图输出坐标。

---

# Phase 4：抽离 Robot Geometry

不要使用 `calculate_self` 这个名字。

新增：

```text
robot_geometry_core
```

职责只描述“机器人自身几何”。

第一版包含：

```text
RobotFootprint
Polygon2D
Pose2D
FootprintTransformer
RobotDimensions（可选）
```

它不负责：

```text
路径规划
costmap
导航
目标选择
Nav2
ROS
```

对于当前 TurtleBot：

```text
固定 footprint polygon
```

即可。

未来双足可以替换为：

```text
URDF
  ↓
动态机器人姿态
  ↓
DynamicFootprintProvider
```

甚至以后接 FCL / MoveIt 等碰撞系统，上层 Navigation 不需要改接口。

---

# Phase 5：重新定义 Navigation Core

Navigation Core 仍然保留，因为它不是单纯“计算机器人大小”。

最终职责：

```text
navigation_core
├── PathSafetyChecker
├── GoalValidator
├── SingleGoalGate
├── PathMetrics
└── navigation types
```

但必须改成纯 C++：

禁止直接出现：

```text
rclcpp
nav_msgs
geometry_msgs
nav2_costmap_2d
frontier_explorer_ros
```

例如：

原来：

```cpp
bool isSafe(
    const nav_msgs::msg::Path & path,
    ...);
```

改成：

```cpp
bool is_safe(
    const Path2D & path,
    const GridMap & map,
    const RobotFootprint & footprint,
    ...);
```

ROS 层负责：

```text
nav_msgs::Path
      ↓ converter
Path2D

geometry_msgs::PoseStamped
      ↓ converter
Pose2D

Nav2 Costmap
      ↓ adapter
GridMap
```

最终：

```text
NavigationNode
├── Nav2 Action Adapter
├── ROS Message Converter
├── Costmap Adapter
│
└── navigation_core
      ├── PathSafetyChecker
      └── GoalValidator
             ↑
      robot_geometry_core
```

---

# Phase 6：处理 GridMap 公共依赖

当前 Navigation 依赖 `frontier_explorer_ros::CostmapAdapter` 是不合理的。

Frontier 和 Navigation 都需要二维 Grid，因此不要让 Navigation 依赖 Frontier。

建议抽一个非常轻量的纯 C++：

```text
grid_map_core
├── GridMap
├── GridCell
├── world_to_grid()
├── grid_to_world()
└── basic grid access
```

依赖关系：

```text
grid_map_core
   ↑          ↑
Frontier   Navigation
              ↑
      robot_geometry_core
```

把当前 `frontier_explorer_core::GridMap` 迁到这里。

为避免一次性破坏大量代码，可以暂时提供：

```cpp
using GridMap = grid_map_core::GridMap;
```

作为兼容层。

---

# Phase 7：训练数据闭环验收

现有 `exploration_learning` 不推倒重写。

继续保留：

```text
DatasetRecorderNode
EventBuffer
DatasetWriter
IDataRecordPlugin
FrontierDecisionPlugin
```

但必须保证每一个 decision 最终可以关联：

```text
episode_id
decision_id
policy_name
policy_version

robot pose

candidate[]
  candidate_id
  position
  distance
  clearance
  unknown_ratio
  path_length
  reachable
  rule_score

selected_candidate_id

navigation outcome
  success / failed / cancelled
  actual_path_length
  duration
  recovery_count

map before
map after
```

至少能够计算：

```text
delta explored area
delta known cells
navigation success
path cost
time cost
```

运行时暂时不要强行固定 Reward 公式。

Reward 放到离线训练阶段计算，这样后续可以反复调整：

```text
reward =
  + exploration gain
  - path cost
  - time cost
  - recovery penalty
  - failure penalty
```

而不用重新采数据。

---

# Phase 8：第一版训练目标

第一版不是 RL。

先做监督/排序学习：

```text
候选 Frontier
       ↓
candidate features
       ↓
Learned Ranker
       ↓
预测 candidate utility
       ↓
排序
```

RuleBasedFrontierRanker 继续作为 baseline。

比较：

```text
RuleBased
vs
Learned
```

指标：

```text
单位时间探索面积
单位路径探索面积
导航成功率
平均 recovery 次数
总探索完成时间
```

模型验证成功以后，再考虑：

```text
Learned Exploration Policy
        ↓
RL / Offline RL
```

---

# 执行顺序

严格按以下顺序提交，不要一个 commit 全改完：

```text
P1
新增 grid_map_core

P2
新增 exploration_core + IExplorationPolicy

P3
FrontierExplorationPolicy 接入现有 Frontier 算法

P4
新增 IFrontierRanker
现有算法包装成 RuleBasedFrontierRanker

P5
新增 robot_geometry_core

P6
navigation_core 去 ROS/Nav2 依赖

P7
Node / Adapter 层完成 ROS ↔ Core 转换

P8
检查 exploration_learning 数据闭环

P9
补 dataset schema validator / training dataset exporter

P10
跑完整 Gazebo 回归
```

每个 P 都要求：

```text
build 通过
unit test 通过
旧行为不变
再进入下一步
```

---

# 验收标准

完成后至少满足：

```text
exploration_core
    ROS-free

frontier_explorer_core
    ROS-free
    实现 IExplorationPolicy

navigation_core
    ROS-free

robot_geometry_core
    ROS-free

grid_map_core
    ROS-free
```

并且依赖关系必须保持：

```text
ROS / Nav2
   ↓
Adapters / Nodes
   ↓
Core
```

绝对禁止 Core 反向依赖 ROS Adapter。

最终一次完整探索应该产生：

```text
Map
 ↓
FrontierPolicy
 ↓
Decision
 ↓
Navigate
 ↓
Outcome
 ↓
Dataset
```

能够从 Dataset 中通过：

```text
episode_id + decision_id
```

完整恢复一次：

```text
当时看到了什么
有哪些候选
为什么选择这个候选
最后结果怎么样
```

达到这一条，就可以正式开始第一版机器学习训练。
