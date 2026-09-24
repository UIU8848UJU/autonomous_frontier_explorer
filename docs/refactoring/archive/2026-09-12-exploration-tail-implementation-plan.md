# 探索收尾与连续选点重构实施计划

> 归档说明：该计划记录 2026-09-12 的分阶段实施要求。
>
> 代码仓库：仓库根目录
>
> Ubuntu 测试工作区：`<workspace>`
>
> 依据文档：[探索收尾与选点性能分析](2026-09-12-exploration-tail-analysis.md)
>
> 实施结果已进入后续提交，当前代码参考 `d7be6c6`（2026-09-24）。

## 1. 改造目标

本次改造解决两个相互独立的问题：

1. 探索结束时仍残留可达的小块未知区域，部分 frontier 没有进入最终候选集合。
2. 每次导航到达后，机器人停在原地完成下一轮候选计算和路径可行性检查，目标衔接不流畅。

目标结果：

- 不再把“frontier 被规则过滤光了”误判成“探索完成”。
- 单格、小型、靠近地图边缘但安全可达的 frontier 能进入收尾流程。
- 所有候选被拒绝时可以明确知道死在哪一道门禁。
- 下一目标的主要计算与当前导航过程重叠，减少两个导航目标之间的零速度空窗。
- 保持 Core 为纯 C++，ROS、Nav2、日志、计时和异步调度仍位于 Adapter/Node/BT 层。

## 2. 本次范围

### 2.1 范围内

- frontier 检测边界修正；
- 小 cluster 的正常模式/收尾模式分级处理；
- 候选门禁诊断；
- 完成判定去除单次 `clusters.empty()` 直接完成；
- cluster/goal 失败语义修正；
- Top-K 可行性检查；
- 导航期间预取下一批候选；
- 到达前或到达后快速复核预取目标；
- 参数 YAML 化；
- Core、ROS Adapter、Node、BT 单元测试和 Ubuntu 模块测试。

### 2.2 范围外

- 不引入 ML/RL Ranker；
- 不实现完整 Active SLAM；
- 不把 2D GridMap 改成 3D voxel map；
- 不修改 SLAM Toolbox、Nav2 planner/controller 内部实现；
- 不以 `explored_ratio` 单独决定探索完成；
- 不在一次提交中同时完成观测、正确性、性能三类改造；
- 不提交 `.gitignore`、临时记录、构建产物或其他无关文件。

## 3. 当前问题链路

```text
/map + global costmap
        ↓
FrontierDetector
        ↓
FrontierPruner
  尺寸 / 距离 / 未知率 / footprint / 黑名单 / 去重
        ↓
RuleBasedFrontierRanker
        ↓
GetFrontierCandidates
        ↓
BT 串行检查最多 16 个候选的 ComputePathToPose
        ↓
NavigateToFrontier
        ↓
导航完成后重新开始整轮计算
```

关键错误语义：

```text
当前：没有检测到 cluster → COMPLETED

正确：
没有检测到 cluster
  → 检查地图是否稳定
  → 检查是否存在剩余未知连通域
  → 检查是否只是边界/尺寸/安全门禁导致
  → 连续满足完成条件
  → COMPLETED
```

## 4. 总体实施原则

1. **先观测，后改行为。** 没有拒绝原因和耗时基线时，不允许直接调参宣称问题已修复。
2. **安全约束不放松。** cleanup 可以放宽尺寸和未知率策略，但不得绕过 footprint、障碍和路径安全。
3. **门禁与排序分离。** Detector/Pruner 决定候选是否存在，Ranker 只决定候选优先级。
4. **完成、无候选、不可达分开。** 三者不得继续复用同一状态。
5. **边跑边选只做计算重叠。** 任意时刻仍只允许一个活动导航目标。
6. **Core 不依赖 ROS。** Core 内使用 enum/struct；ROS message、RCLCPP 日志、时间戳和 service/action 只在 ROS 层出现。
7. **所有新阈值 YAML 化。** 禁止新增不可解释的算法硬编码。
8. **中文注释。** 注释解释原因、边界和不变量，不逐行翻译代码。

## 5. 阶段 A：建立诊断和性能基线

### 5.1 目标

在不改变现有候选选择结果的前提下，能够回答：

- 每轮找到了多少 frontier cell、cluster 和候选？
- 每个候选或 cluster 被哪条规则拒绝？
- 选点时间花在 Detector、Pruner、Ranker 还是 Nav2 planner？
- 两个导航目标之间停了多久？

### 5.2 Core 诊断模型

在 `frontier_strategy_core` 增加纯 C++ 诊断类型，建议职责如下：

```cpp
enum class FrontierRejectionReason
{
  MAP_BOUNDARY,
  CLUSTER_TOO_SMALL,
  GOAL_TOO_CLOSE,
  MAP_CELL_INVALID,
  UNKNOWN_RATIO_TOO_HIGH,
  SAFETY_REJECTED,
  SAME_AS_LAST_GOAL,
  GOAL_RETRY_EXHAUSTED,
  GOAL_BLACKLISTED,
  CLUSTER_RETRY_EXHAUSTED,
  CLUSTER_BLACKLISTED,
  DUPLICATE_GOAL,
  NO_CANDIDATE_GENERATED
};

struct FrontierDecisionDiagnostics
{
  std::size_t raw_frontier_cells;
  std::size_t raw_clusters;
  std::size_t generated_candidates;
  std::map<FrontierRejectionReason, std::size_t> rejection_counts;
};
```

名称可以按现有风格调整，但必须满足：

- Core 类型不引用 ROS message、rclcpp、Nav2；
- 每个失败分支记录唯一、稳定、可测试的原因；
- 诊断只旁路记录，不改变原有返回结果；
- `FrontierStrategyEvaluation` 能携带本轮诊断摘要。

### 5.3 ROS/BT 性能诊断

在 ROS/BT 层记录以下时间点：

```text
candidate_request_started
candidate_response_received
feasibility_candidate_started(index)
feasibility_candidate_finished(index)
feasibility_selection_finished
navigation_goal_sent
navigation_goal_accepted
navigation_result_received
```

至少输出以下指标：

- `frontier_detection_ms`
- `frontier_pruning_ms`
- `frontier_ranking_ms`
- `candidate_service_total_ms`
- `feasibility_check_ms[index]`
- `feasibility_selection_total_ms`
- `navigation_gap_ms`：上一目标 result 到下一目标 accepted

优先复用现有 `/exploration/decision_debug_json`；不要为了第一阶段新增大量 ROS msg。标准运行日志使用 `RCLCPP_DEBUG/INFO/WARN`。

### 5.4 阶段 A 测试

- 每种拒绝分支至少一个 Core 单测；
- 同一输入改造前后候选结果完全一致；
- 空地图、无效地图、单格 frontier、边缘未知区、全候选拒绝均有诊断；
- BT 测试能验证候选检查次数和导航空窗计时事件顺序；
- 不要求阶段 A 改善覆盖率或停车时间。

### 5.5 阶段 A 提交

建议提交信息：

```text
feat(exploration): add frontier rejection and selection timing diagnostics
```

### 5.6 Gate A

在 Ubuntu 至少运行一次能复现收尾问题的场景，并保存：

- 完整 decision debug；
- 每类拒绝数量；
- 每候选 planner 耗时；
- `navigation_gap_ms`；
- 最终地图截图或 rosbag；
- 剩余未知区域位置。

没有这些证据，阶段 B 不得通过验收。

## 6. 阶段 B：修复探索收尾正确性

### 6.1 Detector 边界修复

需求 `EXP-COVER-001`：Detector 必须检查地图内全部网格，而不是跳过最外圈。

行为约束：

- frontier 仍定义为“地图内自由单元，且邻接至少一个地图内未知单元”；
- 地图外区域不作为未知单元；
- 安全邻域越过地图边界时，只跳过地图外采样，不得直接把候选判为障碍；
- 地图内障碍仍然必须拒绝；
- 遍历和邻域访问不得越界。

验收：

- 地图最外圈自由格邻接地图内未知格时，可被检测为 frontier；
- 邻域部分越界但地图内无障碍时，不因越界被拒绝；
- 邻域存在地图内障碍时仍被拒绝。

### 6.2 正常模式与 cleanup 模式

增加纯 C++ 模式语义：

```text
NORMAL
CLEANUP
```

建议参数：

```yaml
frontier_decision:
  normal_min_cluster_size: 2
  cleanup_min_cluster_size: 1
  cleanup_trigger_no_candidate_cycles: 3
  cleanup_trigger_only_small_clusters: true
  cleanup_candidate_max_unknown_ratio: 0.40
  cleanup_exit_progress_ratio_delta: 0.01
```

参数默认值需要结合 Gate A 数据确认；上面数值只是初始建议，不是未经测试的最终指标。

需求 `EXP-COVER-002`：

- NORMAL 优先处理正常 cluster；
- 当连续若干轮无候选但仍有原始 cluster，或者只剩小 cluster 时进入 CLEANUP；
- CLEANUP 接受单格 cluster，并使用独立的未知率阈值；
- CLEANUP 不得关闭 footprint、障碍、路径安全检查；
- 地图出现明显新区域或 cleanup 目标成功带来进度后，可以回到 NORMAL；
- 模式切换必须带原因日志。

### 6.3 候选生成与信息增益

本阶段不要求完整 3D NBV，但 2D 候选必须从“固定几何点”向“观测位姿”靠拢。

最低要求：

- 候选位于已知自由空间；
- 候选朝向目标未知连通域；
- 使用传感器量程和二维射线/可见性估计预计可观察未知单元数；
- 信息增益以预计可见未知单元数为主，不再仅使用候选周围方窗未知率；
- 固定的退让距离、采样半径和角度步长全部参数化；
- 每个未知连通域至少尝试多个观测位姿，而不是只依赖 centroid。

建议新增配置：

```yaml
frontier_decision:
  sensor_range_m: <按实际雷达填写>
  viewpoint_sample_radii_m: [0.25, 0.40, 0.55]
  viewpoint_angle_step_deg: 30.0
  minimum_visible_unknown_cells: 1
  information_gain_ray_step_cells: 1
```

如果实际传感器量程无法从现有配置确认，先保留参数且由用户填写，不得猜测硬件数据。

### 6.4 失败与黑名单语义

需求 `EXP-COVER-003`：候选生成失败不得等同于导航失败。

分开维护：

```text
candidate_generation_rejection
planner_unreachable
footprint_rejected
path_unsafe
navigation_failed
```

规则：

- 本轮未生成候选只能计入诊断，不得立即永久拉黑 cluster；
- goal/cluster 黑名单需要 TTL 或地图版本失效机制；
- 地图发生实质变化后允许重新评估旧黑名单；
- 黑名单优先按空间半径/区域表达，不能只依赖精确 GridCell；
- cluster 标识不得只依赖会漂移的 centroid；建议使用地图版本内稳定的连通域 ID 或量化空间区域 ID；
- `max_frontier_failures` 要么真正参与策略，要么删除，禁止保留“配置存在但行为不生效”的参数。

### 6.5 完成判定

需求 `EXP-COVER-004`：删除 `clusters.empty() -> COMPLETED` 的单次直接映射。

最小可接受完成条件：

```text
连续 N 个稳定地图版本
AND 无原始有效 frontier
AND 无 cleanup 候选
AND 剩余未知连通域均已分类
→ COMPLETED
```

剩余未知域分类至少包括：

```text
MAP_BOUNDARY_OUTSIDE_SCOPE
OBSTACLE_ENCLOSED
UNREACHABLE
LOW_INFORMATION_NOISE
PENDING_RECHECK
```

约束：

- `explored_ratio` 只能用于进度、停滞检测和日志；
- `STUCK`、`NO_VALID_CANDIDATE`、`COMPLETED` 必须是不同语义；
- 地图持续变化期间不能完成；
- 任何仍有可达且满足最低信息增益的观测位姿时不能完成。

建议参数：

```yaml
completion:
  stable_no_frontier_cycles: 3
  stable_map_duration_ms: 2000
  ignored_unknown_component_max_cells: 1
```

最终默认值必须由 Gate A 场景验证。

### 6.6 阶段 B 必测场景

1. 单格未知孤岛附近存在安全观测位姿；
2. 地图边缘存在地图内未知区域；
3. 窄门后方有小区域；
4. 未知区域完全被障碍封闭；
5. 当前 costmap 暂时未更新，下一地图版本恢复；
6. 所有候选 footprint 冲突；
7. 有 cluster 但 planner 不可达；
8. 地图连续更新时不得提前完成；
9. 连续稳定且所有剩余未知域已分类时才完成。

### 6.7 阶段 B 提交

建议拆为两个提交：

```text
fix(exploration): preserve small and boundary frontiers for cleanup
fix(exploration): make completion depend on stable reachable coverage
```

### 6.8 Gate B

必须满足：

- 不再出现“仍存在可达未知连通域但状态为 COMPLETED”；
- 单格和边缘 frontier 单测通过；
- cleanup 至少成功处理一个原来遗漏的小区域；
- 不可达封闭区域不会导致无限循环；
- 正常探索路径没有因 cleanup 参数而明显退化；
- 所有状态变化有原因日志。

## 7. 阶段 C：减少停车选点时间

### 7.1 先实现 Top-K，不先做无限并发

需求 `EXP-PERF-001`：不再固定对最多 16 个候选全部执行 planner 检查。

建议配置：

```yaml
selection:
  feasibility_top_k: 3
  feasibility_early_accept_utility: <由基线确定>
  feasibility_cache_enabled: true
  feasibility_cache_map_revision_ttl: 1
```

行为：

- RuleRanker 先做低成本排序；
- 默认只检查前 K 个候选；
- 如果候选达到可接受效用阈值，可以早停；
- 如果 Top-K 全部不可行，按批次继续检查后续候选，而不是立即失败；
- 第一版保持有限串行，确认 Nav2 planner 并发能力后才考虑 2 路有限并发；
- 不允许无上限并发调用 `ComputePathToPose`。

### 7.2 可行性缓存

需求 `EXP-PERF-002`：缓存 planner 结果，避免同一地图状态反复规划相同目标。

缓存键至少包含：

```text
map_revision / costmap_revision
robot_start_region
goal_region
planner_id
```

失效条件：

- 地图或 costmap 发生实质更新；
- 机器人离缓存起点超过阈值；
- planner_id 改变；
- 缓存超过 TTL；
- 上一次执行路径出现阻塞或导航失败。

### 7.3 导航期间预取

需求 `EXP-PERF-003`：当前目标导航时允许后台准备下一目标。

推荐数据流：

```text
ACTIVE_GOAL 正在导航
        ├─ 地图持续更新
        └─ Prefetch 生成/排序下一批候选
                 ↓
          PREFETCHED_CANDIDATES
                 ↓ 当前目标接近完成
          用最新位姿/costmap复核 Top-K
                 ↓ 当前目标结束
          立即发送已复核的下一目标
```

必须保证：

- 任意时刻只有一个活动导航 goal；
- 预取数据带 map/costmap revision 和生成时机器人位姿；
- 旧 revision 的候选不能直接执行；
- 当前正在导航的目标及其邻域不能再次成为预取目标；
- 导航失败时预取结果需要重新验证；
- 异步回调共享字段必须使用 mutex/atomic 或统一 executor 所有权，禁止继续使用无同步普通 bool；
- 停止请求能够取消当前 goal，并使预取任务失效。

### 7.4 行驶中目标替换

第一版默认关闭。只有预取稳定后才允许通过参数启用：

```yaml
selection:
  enable_active_goal_replacement: false
  goal_switch_min_utility_gain: <由实验确定>
  goal_min_hold_duration_ms: <由实验确定>
```

即使启用，也必须同时满足：

- 新目标效用显著高于当前目标；
- 当前目标已保持最短时间；
- 新目标已通过最新安全和可达性检查；
- 不处于当前目标的到达容差内；
- 单位时间内切换次数不超过限制。

### 7.5 阶段 C 验收

- `navigation_gap_ms` P50/P95 相比阶段 A 基线显著降低；
- 初始建议目标：P95 小于 0.5～1.0 秒，最终以 Ubuntu 实测基线确认；
- 平均 planner 调用次数明显下降；
- 不降低阶段 B 的地图覆盖结果；
- 不出现并发导航目标；
- 不出现高频 cancel/re-goal、路径振荡或候选抖动；
- 服务不可用、预取过期、地图突变时能退回普通同步选择流程。

### 7.6 阶段 C 提交

建议拆为：

```text
perf(exploration): bound frontier feasibility checks with cache
perf(exploration): prefetch next frontier during navigation
```

## 8. 测试矩阵

| 测试层 | 必测内容 | 运行位置 |
|---|---|---|
| Core 单元测试 | 边界、单格 cluster、拒绝原因、cleanup、完成策略、缓存键 | Windows 可做静态检查，Ubuntu 正式执行 |
| ROS Adapter 测试 | 参数转换、诊断转换、地图版本、黑名单失效 | Ubuntu |
| BT 单元/组件测试 | Top-K、批次回退、预取失效、停止、异步并发安全 | Ubuntu |
| 模块测试 | exploration_core、frontier_strategy_core/ros、exploration_nodes、exploration_bt | Ubuntu `<workspace>` |
| 场景测试 | 开阔区、窄门、小未知区、边缘区域、封闭不可达区 | Ubuntu 仿真/实机 |
| 集成测试 | 完整 SLAM→探索→地图保存 | 用户执行 |

所有阶段至少执行：

```bash
colcon build --packages-up-to \
  exploration_core \
  frontier_strategy_core \
  frontier_strategy_ros \
  navigation_core \
  exploration_nodes \
  exploration_bt

colcon test --packages-select \
  exploration_core \
  frontier_strategy_core \
  frontier_strategy_ros \
  navigation_core \
  exploration_nodes \
  exploration_bt

colcon test-result --verbose
```

实际包名如果与当前仓库不同，以 `colcon list` 为准，不得静默跳过失败包。

## 9. 文件级改动指引

优先检查和修改：

```text
src/frontier_strategy_core/
  src/detector/frontier_detector.cpp
  src/selector/frontier_pruner.cpp
  src/selector/frontier_selection_policy.cpp
  src/policy/frontier_strategy_policy.cpp
  include/.../types/
  test/

src/frontier_strategy_ros/
  src/frontier_goal_provider.cpp
  include/frontier_strategy_ros/frontier_goal_provider.hpp
  test/

src/exploration_nodes/
  src/frontier_strategy_node.cpp
  src/navigation_node.cpp
  config/frontier_strategy.yaml
  test/

src/exploration_bt/
  behavior_trees/exploration_tree.xml
  src/bt/action/compute_frontier_candidates_action.cpp
  src/bt/action/select_feasible_frontier_action.cpp
  src/bt/action/navigate_to_frontier_action.cpp
  src/bt/orchestrator/exploration_bt_orchestrator_node.cpp
  config/exploration_bt.yaml
  test/
```

不要因为文件列在这里就机械地全部修改；每个提交只动该阶段需要的文件。

## 10. 兼容和回滚

- 新行为参数必须提供保守默认值；
- 阶段 A 不得改变候选输出；
- cleanup 和预取均应可通过 YAML 单独关闭；
- 预取失败时回退当前同步流程；
- 不修改现有 ROS service/action 名称，除非单独提出接口迁移；
- 如果必须修改 `robot_interfaces`，需要同步更新所有 producer、consumer 和测试，并单独说明兼容影响；
- 每个阶段独立提交，允许只回滚性能优化而保留正确性修复。

## 11. Git 与工作区约束

当前 Windows 工作区已有未提交重构改动。Luna 开始前必须：

1. 执行 `git status --short` 和 `git diff --stat`；
2. 不执行 `git reset --hard`、`git checkout -- <file>` 或覆盖用户现有改动；
3. 只暂存本计划相关代码、配置、测试和必要文档；
4. 不暂存无关 `.gitignore`、`issue.md`、日志、bag、截图、build/install/log；
5. 每阶段提交前展示 staged diff；
6. 未经用户明确要求不 push；
7. 向 Ubuntu 同步时只覆盖源码相关内容，不删除远端工作区未知文件。

## 12. Luna 每阶段交付格式

每个阶段完成后必须报告：

```text
阶段：A / B / C
完成需求 ID：
修改文件：
行为变化：
新增参数及默认值：
新增测试：
Windows 检查结果：
Ubuntu build/test 结果：
场景证据：
已知风险：
是否满足当前 Gate：PASS / CONDITIONAL / FAIL
提交哈希：
未提交无关文件：
```

## 13. 开发准入结论

### 允许立即执行

- 阶段 A：诊断与基线，范围和验收已经明确。

### 有条件执行

- 阶段 B：需要 Gate A 证明主要拒绝原因，并用数据确认 cleanup 参数默认值。
- 阶段 C：需要阶段 B 保证完成语义和覆盖率正确，避免只把错误决策执行得更快。

### 暂不执行

- ML/RL Ranker；
- 完整 Active SLAM；
- 3D NBV；
- 多机器人全局任务分配。

本计划的完成定义不是“代码能编译”，而是：能够解释每个残留区域的处理结果、不再假完成，并且目标衔接时间有可复现的量化改善。
