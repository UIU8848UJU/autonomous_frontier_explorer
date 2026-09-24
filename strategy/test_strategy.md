# A/B/C 最小可靠测试策略

> 归档说明：本文是 2026-09-12 实施前的测试准入设计，保留用于追溯风险与验收思路。
>
> 当前代码参考：`d7be6c6`（2026-09-24）。下述 Gate 状态和环境判断是历史快照，不代表当前实现状态。

## 独立审查判断

当前计划的方向正确，但“测试矩阵有列名”还不等于 Gate 可验收。建议决策为：

- 阶段 A：`CONDITIONAL_GO`。先冻结 baseline fixture、输出比较器、trace 契约和 Ubuntu 复现场景；完成后才可进入 B。
- 阶段 B：`BLOCKED_PENDING_A`。必须先有 A 的拒绝原因与耗时证据，否则 cleanup 参数和 completion 稳定周期没有依据。
- 阶段 C：`BLOCKED_PENDING_B`。必须先证明 B 不假完成、失败语义正确且覆盖不退化，再优化调用次数和 gap。

## 最小测试层级

### 1. Core Unit/Property：必须快速、确定

覆盖 detector 边界、邻域越界、free/unknown/obstacle 语义、cluster 聚类、cleanup mode 转移、completion truth table、未知域分类、blacklist key/TTL、cache key/invalidation、Top-K/batch 算法和 ray visibility。使用小型固定 GridMap，不引入 ROS。

最小断言不是“返回非空”，而是具体集合、状态、原因、计数和不变量：

- 地图外采样永远不作为未知或障碍；地图内障碍仍拒绝。
- `raw_cells`、`raw_clusters`、`generated_candidates` 的定义和计数可由 fixture 手算。
- unchecked feasibility 不能被当作 feasible/reachable。
- 完成判定对一次空结果、连续稳定结果、地图变化、可达 cleanup candidate 的结果不同。

### 2. Component：单进程真实 Core + fake 外部依赖

使用真实 Policy/Pruner/Ranker/Coordinator，注入 fake clock、map revision、pose、planner、navigation/service client 和受控 callback scheduler。它是验证 Top-K、缓存、预取、失败分层和 BT flow 的主层级。

fake 必须可记录：调用顺序、请求内容、revision、start region、goal region、planner id、返回延迟、cancel 次数和活动 goal 数。任何 fake 结果都只能证明协调器逻辑，不能证明 Nav2 planner 真实可达性。

### 3. Contract/ROS Adapter：验证边界而非重复算法

验证 YAML → normalized params、OccupancyGrid → GridMap、候选并行数组长度/索引一致、reason/state/detail 映射、map/costmap revision 传递、服务/action timeout/recoverable 语义。对 JSON debug 至少做结构化解析和字段存在性校验，不能只搜日志字符串。

### 4. Integration：Ubuntu ROS executor + fake/real action boundary

至少启动 FrontierStrategyNode、Exploration BT、NavigationNode 和受控 fake Nav2 action server，验证 service/action/executor、stop/cancel、TF/costmap 更新和单活动 goal。对真实 Nav2 ComputePathToPose 做有限场景测试，验证 planner reject/timeout/path unsafe 与 fake 结论一致的边界。

### 5. 有限 E2E/Replay：只覆盖关键业务链路

不以 E2E 替代 Unit。最小场景为：开阔正常区、边缘未知区、单格/窄门小区、障碍封闭未知区、planner 不可达区、地图暂不更新后恢复。使用版本化地图、YAML、机器人初始 pose、Nav2/SLAM 版本和 bag/trace 目录。

## 阶段 A 进入与 Gate A

### 最小切片

1. A-01：诊断 on/off 输出完全等价。
2. A-02：拒绝原因稳定且可按候选/cluster 追溯。
3. A-03：事件 trace 能计算 detector/pruner/ranker/planner/navigation gap。
4. A-04：Ubuntu 复现原收尾问题。

### Gate A 必需证据

- baseline 的 commit/工作树摘要、Ubuntu/ROS/Nav2/编译器版本、YAML 完整快照；
- 场景地图、costmap、初始 pose 的 hash；若是 SLAM，保存 rosbag 或可重放输入；
- 至少一次成功复现收尾问题；为使 P95 和拒绝频率可解释，建议同配置同场景完成 3 次 run，单次至少记录完整收尾周期；
- 每次 run 的原始 decision debug、结构化 DecisionTrace、各拒绝原因、raw cells/clusters/candidates、每候选 planner 耗时、navigation gap、最终未知区域截图/地图；
- “诊断 on/off 候选结果相等”的自动比较结果；
- 说明 `raw_frontier_count` 到底代表 cell 还是 cluster，并确认下游没有静默接受错误语义。

Gate A 不要求改善停车时间，但要求数据足以回答“时间和候选为什么失败”。缺少 raw 输入或只提供截图，判定为 `CONDITIONAL` 而不是 PASS。

## 阶段 B 进入与 Gate B

### 先后顺序

Detector 边界 → mode-aware pruner → viewpoint/ray gain → failure/blacklist lifecycle → unknown classification → completion state machine → ROS/BT mapping → E2E。

### Gate B 必需证据

- B-01/B-02 所有边缘和障碍真值表通过，包含最小宽高地图和 obstacle radius > 0；
- 同一单格 cluster 在 NORMAL/CLEANUP 的差异可观察，且 footprint、障碍、path safety 约束在 cleanup 不变；
- 单格、边缘、窄门场景至少一个真实可达小区域被处理，而不是仅返回 candidate；应有机器人轨迹/导航结果和地图变化证据；
- unknown components 的每一种分类都有输入、判定依据和输出；不可达封闭区有有界重试，不进入无限循环；
- 一次 `clusters.empty()`、连续稳定无 frontier、地图持续更新、下一版本恢复、仍有可达 cleanup candidate、全候选安全冲突、planner 不可达分别得到 COMPLETED/STUCK/WAIT/NO_VALID 的预期状态；
- 完成状态连续 N 个稳定版本的计数、稳定时长、版本变化和分类集合均出现在 trace；
- B 场景回归未出现可达 unknown 被 COMPLETED、cleanup 绕过 footprint/path safety、正常 cluster 被 cleanup 参数吞没。

Gate B 的“完成”必须是 L2 + L4 联合证据：Core 状态机证明逻辑，Ubuntu 场景证明地图/TF/Nav2 的真实边界。

## 阶段 C 进入与 Gate C

### 最小切片

1. C-01/C-02：Top-K、早停、失败后批次回退。
2. C-03：cache key 与所有失效条件。
3. C-04/C-05：prefetch revision/pose capture、最新复核、stop/failure fallback。
4. C-06/C-07：单活动 goal 和并发安全。
5. C-08：量化 gap 改善且覆盖不退化。

### Gate C 必需证据

- scripted component trace 证明只检查 K、unchecked 不可选、后续批次可达；
- cache hit/miss call count 与 revision/start/planner/TTL/failure truth table 完全一致；
- 注入地图突变、costmap 突变、机器人位姿移动、stop、nav failure、planner timeout、late callback，均无旧 prefetch 被发送或写回；
- fake action server 和 Ubuntu integration 均证明 `active_navigation_count <= 1`，且 cancel 后最终收到且只收到一个 terminal result；
- Linux TSan 或等价线程检查 + 受控 callback stress；没有 TSan/竞态证据，不得以“加了 mutex”关闭 C 并发风险；
- baseline 与 C 使用同一场景/配置/硬件，至少 30 个目标衔接样本、3 次 run，报告 P50/P95 `navigation_gap_ms`、planner 调用次数、cancel/re-goal 次数；
- 同时报告覆盖率代理（最终未知域/地图变化/完成判定）、安全拒绝和失败恢复，不能只报告速度。

如果 P95 样本少于 20，只能报告描述性数据，不得把阈值当作统计验收。

## Flaky 与失败政策

- 时间测试不直接断言单次绝对毫秒值；逻辑层用 fake clock，性能层用固定样本和同机对照。
- ROS action/service 测试失败时保留 trace、executor 调度、版本和原始响应；重跑仍失败才进入缺陷。
- 同一测试两次非确定失败视为 flaky 候选，必须隔离/修复；不能通过提高重试次数掩盖旧 callback 或状态污染。
- 任何测试跳过必须在报告写出原因、影响和残余风险；`colcon test-result --verbose` 中包缺失或测试未注册视为失败，不是通过。

## 当前执行限制

制定本文时只有 Windows 源码工作区，没有 colcon、ROS、build/install/log 产物；因此当时只能给出静态审查和测试设计，不能据此声称 Ubuntu module/integration/E2E 已通过。计划中的 Ubuntu `<workspace>` 是必要运行环境，不应被 Windows 编译或现有 gtest 数量替代。
