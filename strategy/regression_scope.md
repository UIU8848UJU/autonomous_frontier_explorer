# 回归范围与裁剪项

## 每个阶段都必须运行

- `exploration_core`：决策/结果枚举和 ROS-free contract；确认 COMPLETED、STUCK、WAIT、FAILED 语义未被混淆。
- `frontier_strategy_core`：Detector、Pruner、Ranker、SelectionPolicy、Policy 全部现有单测及新增 fixture/property tests。
- `frontier_strategy_ros`：OccupancyGrid 转换、坐标/分辨率、global costmap safety、参数转换、候选数组 contract。
- `navigation_core`：footprint/path safety 和 `SingleGoalGate`；C 的 prefetch 不得绕过该安全边界。
- `exploration_nodes`：FrontierGoalProvider、NavigationNode 的 timeout/recoverable/footprint/path safety、debug JSON 和 state mapping。
- `task_manager`：收到 COMPLETED/STUCK/FAILED 的状态转移；防止 B 把暂时无候选送成完成，或造成错误地图保存。
- `robot_interfaces`：除非确有必要，不改 service/action 名称；若新增 revision/classification 字段，必须做 producer/consumer 双向 contract。

## 阶段 A 回归

重点跑：现有 core/ROS frontier tests、invalid map、正常候选顺序、blacklist/retry、provider service response、debug JSON schema、no-ROS dependency check。新增 diagnostics off/on golden comparison。

不可用的证据：只看日志中出现了某个 reason；只看 coverage 百分比；只在 fake planner 下声称 Nav2 planner 耗时准确。

## 阶段 B 回归

扩大到：所有 frontier map fixtures、global costmap footprint/path safety、TF/map stale recovery、FrontierStrategyNode state、BT completion path、TaskManager COMPLETED/STUCK、原收尾场景 replay。

必须包含反例：

- map 内还有一个可达未知连通域；
- 只有一个安全冲突候选；
- planner 不可达但仍有其他候选；
- 地图正在变化；
- centroid 变化但区域仍是同一未知域；
- 全部剩余 unknown 是封闭障碍或已分类噪声。

## 阶段 C 回归

在 B 全量范围上增加：候选服务数组/上限、Top-K/batch、planner cache invalidation、prefetch cancellation、NavigationNode SingleGoalGate、BT reload、stop/start、TSan/stress、性能对照和 fallback。

性能优化只允许在安全/完成语义回归全通过后验收；任何 coverage 下降、假完成、双活动 goal、旧 revision goal 执行或高频 cancel/re-goal，均使 C Gate FAIL。

## 裁剪项与残余风险

| 裁剪项 | 原因 | 残余风险 | 处置 |
|---|---|---|---|
| Windows 不跑 ROS/colcon | 目标运行环境为 Ubuntu ROS 2 Humble | 编译、executor、Nav2、TF 行为未知 | Gate 报告显式标记，Ubuntu module/integration 必须补齐 |
| 不在第一版做完整 Active SLAM/3D NBV | 计划范围外 | 2D ray gain 不能代表完整观测收益 | 只验收 2D 可见性契约，记录边界 |
| 不用完整实机全量探索替代单测 | 成本和不可重复性高 | 场景覆盖有限 | 固定 map/replay + 有限仿真/实机关键场景 |
| 不把所有 Nav2 依赖 Mock | 真实 planner/costmap 边界不能被证明 | fake 结果与真实 Nav2 偏差 | 每阶段至少保留真实 integration/E2E 边界 |
| 不以 coverage 百分比作为 Gate | 覆盖率不能证明完成语义/时序 | 某些分支仍可能漏测 | 用 behavior slice、oracle 和 trace 作为主证据 |
| active goal replacement 第一版关闭 | 计划已默认关闭 | 未来启用仍有切换振荡风险 | 保留独立后续切片，不纳入 C 当前 PASS |

## 测试注册检查

所有新增测试必须在对应 CMake/package 中显式注册。当前 `exploration_bt/CMakeLists.txt` 未发现 test target；在它补齐前，BT 相关条款只能判为未覆盖。Gate 运行必须保存 `colcon list`、build 输出、`colcon test-result --verbose`，证明没有静默跳过包或测试。
