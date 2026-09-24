# Exploration 架构收敛修复计划

> 归档说明：该计划对应提交 `67d9591` 后的评审修复阶段。
>
> 实施结果已合入后续提交，当前代码参考 `d7be6c6`（2026-09-24）；命令中的 `<workspace>` 表示任意 ROS 2 工作区。

## 1. 计划目的

本计划针对提交 `67d9591` 之后的架构评审结果进行收敛。

本轮不增加探索功能，不实现 ML/RL 模型，也不改变 ROS topic、service、action 和状态机语义。目标是修正已经确认的构建边界、依赖方向和替换接缝，使当前代码真实表达以下架构：

```text
ROS / Nav2 Nodes
        ↓
ROS Adapter / Converter
        ↓
纯 C++ Core

Frontier 流程：
Detector → Pruner → IFrontierRanker → Selector
```

源码权威副本：

```text
仓库根目录
```

Ubuntu 构建和模块测试环境：

```text
连接方式：项目配置的远程执行通道
workspace: <workspace>
ROS: ROS 2 Humble
```

实施时必须保留用户当前未提交内容：

```text
.gitignore
issue.md
```

除非用户明确要求，否则本轮完成后不要 push。

---

## 2. 评审结论

### 2.1 `IExplorationPolicy`：问题部分成立

确认存在的问题：

- `FrontierGoalProvider` 直接持有 `FrontierStrategyPolicy`；
- 生产路径直接调用 `FrontierStrategyPolicy::evaluate(...)`；
- `IExplorationPolicy::decide(...)` 不在生产主链路；
- `decide()` 的 `ExplorationObservation` 只包含 map、robot cell 和 mapping 状态，无法表达生产路径使用的 safety、clearance 和 reachability 约束；
- 因此当前 `IExplorationPolicy` 会让代码看起来已经支持整套策略替换，但运行时并没有这个能力。

评审中“Ranker 还没有成为替换点”的说法已经过时：

- `IFrontierRanker` 已经存在；
- `FrontierSelectionPolicy` 已通过 `std::shared_ptr<IFrontierRanker>` 调用 Ranker；
- 默认实现已经是 `RuleBasedFrontierRanker`。

仍需修复的是：Ranker 注入只到达 `FrontierSelectionPolicy`，还没有贯通到 `FrontierStrategyPolicy` 和 `FrontierGoalProvider`。

另外，当前 `IFrontierRanker::rank(...)` 仍接收并执行 reachability callback。这个职责不应该交给可替换 Ranker，否则未来的 Learned Ranker 可以忽略回调并绕过可达性约束。Ranker 应只负责排序，可达性验证必须由稳定的 Selection 层统一执行。

### 2.2 `exploration_nodes_common`：问题成立

当前一个 library target 同时编译：

- `frontier_strategy_node.cpp`；
- `navigation_node.cpp`；
- Frontier reachability/marker；
- Navigation converter。

两个 executable 都链接整个 `exploration_nodes_common`，造成 target 级职责重新耦合。

### 2.3 `navigation_core` 构建依赖 ament：问题成立

源码层已经没有 ROS/Nav2 类型，但 `CMakeLists.txt` 和 `package.xml` 仍依赖：

```text
ament_cmake
ament_cmake_gtest
```

因此它目前是“源码 ROS-free”，还不是“独立构建 ROS-free”。

### 2.4 `MapSnapshot` 重复地图模型：问题成立

`map_lifecycle` 自己定义了 `MapSnapshot`，与 `grid_map_core::GridMap` 重复。当前转换链会逐渐形成多个近似地图结构，后续加入 origin 等字段时容易发生数据丢失和语义分叉。

### 2.5 Frontier Core 四个动态库：事实成立，但优先级为 P3

当前生成：

```text
frontier_strategy_detection_core.so
frontier_strategy_scoring_core.so
frontier_strategy_selection_core.so
frontier_strategy_policy_core.so
```

这些模块没有独立部署、独立版本或稳定 ABI 插件需求。保留类和目录分层即可，现阶段一个 `frontier_strategy_core` library 更合适。

---

## 3. 最终架构决策

### ADR-01：暂不抽象整套 Exploration Policy

本阶段不把 `FrontierGoalProvider` 改成依赖 `IExplorationPolicy`，也不扩大 `ExplorationObservation` 去容纳 ROS/Nav2 环境回调。

执行以下收敛：

- 删除当前没有生产调用者的 `IExplorationPolicy`；
- 删除只为该接口服务、且没有生产用途的 `ExplorationObservation`；
- `FrontierStrategyPolicy` 不再继承整套探索策略接口；
- 删除与生产路径能力不一致的 `FrontierStrategyPolicy::decide(...)`；
- 保留生产实际使用的 `FrontierStrategyPolicy::evaluate(...)`；
- `exploration_core` 继续承载跨策略可复用的状态、Decision 和 Outcome 数据语义。

以后只有在真正出现第二种完整探索策略，并且执行器能够只依赖统一 Observation/Decision/Outcome 契约时，才重新引入 `IExplorationPolicy`。

### ADR-02：当前唯一算法替换点是 Ranker

稳定不替换：

```text
FrontierDetector
FrontierPruner
安全约束
Reachability 约束
FrontierSelectionPolicy
```

允许替换：

```text
IFrontierRanker
├── RuleBasedFrontierRanker
└── LearnedFrontierRanker  # future
```

Ranker 注入链必须贯通：

```text
FrontierGoalProvider
        ↓ constructor/factory injection
FrontierStrategyPolicy
        ↓
FrontierSelectionPolicy
        ↓
IFrontierRanker
```

接口职责应收敛为：

```text
IFrontierRanker
输入：已经通过硬安全剪枝的候选特征、last goal
输出：带 score 的有序候选

FrontierSelectionPolicy
接收 Ranker 输出
统一执行 reachability callback
根据 require_reachable_goal 过滤不可达候选
选择最终目标
```

`IFrontierRanker` 不再接收 reachability callback。ML 排序器不能决定是否执行安全或可达性约束。

Node 的默认生产配置仍使用 `RuleBasedFrontierRanker`，本轮不增加模型加载、插件系统或运行时参数选型。

---

## 4. 实施顺序

每个阶段独立 commit。每完成一个阶段先构建和测试，再进入下一阶段。

### P1：收敛 Exploration Policy，并贯通 Ranker 注入

目标：删除误导性的整套策略替换接口，把现有 Ranker 接缝真正贯通到生产组合根。

主要修改文件：

```text
src/exploration_core/include/exploration_core/policy/exploration_policy.hpp
src/exploration_core/include/exploration_core/types/exploration_observation.hpp
src/exploration_core/CMakeLists.txt
src/exploration_core/test/test_exploration_core.cpp

src/frontier_strategy_core/include/frontier_strategy_core/policy/frontier_strategy_policy.hpp
src/frontier_strategy_core/src/policy/frontier_strategy_policy.cpp
src/frontier_strategy_core/test/test_frontier_strategy_policy.cpp

src/frontier_strategy_ros/include/frontier_strategy_ros/frontier_goal_provider.hpp
src/frontier_strategy_ros/src/frontier_goal_provider.cpp
src/frontier_strategy_ros/test/test_frontier_ros.cpp
```

实施要求：

1. 删除 `IExplorationPolicy` 和 `ExplorationObservation`，同步删除失效测试与 include。
2. `FrontierStrategyPolicy` 保留 `evaluate()`、`reset()` 和结果反馈能力，不再提供能力不足的 `decide()`。
3. 为 `FrontierStrategyPolicy` 增加可选的 `std::shared_ptr<IFrontierRanker>` 构造参数，并传入 `FrontierSelectionPolicy`。
4. 为 `FrontierGoalProvider` 增加可选 Ranker 注入入口；默认空指针时保持规则排序行为。
5. `configure()` 重建 Policy 时必须保留已注入的 Ranker，不能退回默认实现。
6. 从 `IFrontierRanker::rank(...)` 删除 reachability callback，由 `FrontierSelectionPolicy` 在排序后统一执行并写回候选的 reachability 信息。
7. 增加 Fake Ranker 测试，证明候选点生产路径确实经过注入 Ranker，而不是只测试接口可以实例化。

验收：

```text
rg "IExplorationPolicy|ExplorationObservation" src
```

生产代码中应无结果；历史文档如有提及必须同步更新。

Fake Ranker 测试至少证明：

- Ranker 被调用；
- Ranker 能改变候选顺序；
- safety/pruner 仍在 Ranker 之前执行；
- 即使 Fake Ranker 完全不知道 reachability，SelectionPolicy 仍会执行可达性检查并过滤不可达候选；
- `configure()` 后注入对象仍然生效。

建议 commit：

```text
refactor: make frontier ranker the runtime strategy seam
```

### P2：拆分 Exploration Node targets

目标：一个 executable 只链接自己的 Node 实现。

修改：

```text
src/exploration_nodes/CMakeLists.txt
```

目标结构：

```text
frontier_strategy_node_lib
├── frontier_strategy_node.cpp
├── nav2_planner_reachability_checker.cpp
└── frontier_marker_publisher.cpp

navigation_node_lib
├── navigation_node.cpp
└── adapters/navigation_converters.cpp

frontier_strategy_node executable
    → frontier_strategy_node_lib

navigation_node executable
    → navigation_node_lib
```

当前 converter 只被 Navigation 使用，不要为了形式再创建 common helper。以后真有两个 Node 共用的实现时，才建立很小的 helper target。

同时将 target dependencies 分成两组，避免 Frontier target 无条件链接 Navigation/Nav2 action 依赖，也避免 Navigation target 无条件链接 Frontier marker/strategy 依赖。

验收：

- 不再存在 `exploration_nodes_common`；
- 两个 executable 分别只链接自己的 library；
- 删除其中任一 Node library 不应影响另一个 Node library 的源文件编译边界；
- ROS 包整体构建通过。

建议 commit：

```text
refactor: split exploration node implementation targets
```

### P3：将 `navigation_core` 改为普通 CMake package

目标：源码和构建系统都不依赖 ROS/ament。

修改文件：

```text
src/navigation_core/CMakeLists.txt
src/navigation_core/package.xml
src/navigation_core/cmake/navigation_coreConfig.cmake.in       # 新增
src/navigation_core/test/no_ros_dependencies.cmake             # 新增
src/navigation_core/test/downstream_smoke/                      # 建议新增
```

CMake 应参考现有 `grid_map_core`、`robot_geometry_core`：

- 使用 `GNUInstallDirs`；
- 使用 `CMakePackageConfigHelpers`；
- 导出 `navigation_core::navigation_core`；
- 生成 Config 和 ConfigVersion；
- 测试使用标准 `CTest + GTest`；
- package.xml 使用 `<buildtool_depend>cmake</buildtool_depend>`；
- export build type 使用 `<build_type>cmake</build_type>`；
- 删除所有 ament API 和 ament test dependency。

`navigation_coreConfig.cmake.in` 必须查找：

```text
grid_map_core
robot_geometry_core
```

验收：

```text
rg "ament|rclcpp|nav_msgs|geometry_msgs|nav2" src/navigation_core
```

生产源码、CMake 和 package.xml 中应无 ROS/ament 依赖；`no_ros_dependencies.cmake` 中作为禁止词出现不算依赖。

除 colcon 构建外，必须验证标准 CMake 安装和下游 `find_package(navigation_core CONFIG REQUIRED)`。

建议 commit：

```text
refactor: make navigation core a standalone cmake package
```

### P4：Map Lifecycle 统一使用 `grid_map_core::GridMap`

目标：地图基础结构只保留一套。

修改文件：

```text
src/map_lifecycle/include/map_lifecycle_core/map_lifecycle_core.hpp
src/map_lifecycle/core/map_lifecycle_core.cpp
src/map_lifecycle/nodes/map_lifecycle/adapters/ros_message_converter.hpp
src/map_lifecycle/nodes/map_lifecycle/adapters/ros_message_converter.cpp
src/map_lifecycle/test/test_map_lifecycle_core.cpp
src/map_lifecycle/test/test_map_lifecycle_node.cpp
src/map_lifecycle/CMakeLists.txt
src/map_lifecycle/package.xml
```

实施要求：

1. 删除 `MapSnapshot`。
2. `MapLifecycleCore` 的输入和 `latest_map()` 直接使用 `grid_map_core::GridMap`。
3. ROS converter 直接生成 `GridMap`。
4. converter 必须转换 width、height、resolution、data、origin_x、origin_y，不能只复制旧字段。
5. 地图有效性统一使用 `GridMap::isReady()`；补充 resolution 非法和 data 长度不匹配测试。
6. 地图生命周期状态、统计比例、保存触发语义保持不变。
7. `map_lifecycle_core` 公开链接 `grid_map_core::grid_map_core`，并正确导出依赖。

本轮不向 `GridMap` 增加 ROS frame、timestamp，也不把 map I/O 放入 Lifecycle Core。

验收：

```text
rg "MapSnapshot" src
```

应无结果。

必须继续通过：

- 有效地图进入 `ACTIVE`；
-探索完成后进入 `READY`；
- 保存过程为 `SAVING → SAVED/FAILED`；
- 保存失败可重试；
- 地图统计比例保持正确。

建议 commit：

```text
refactor: reuse grid map model in map lifecycle
```

### P5：合并 `frontier_strategy_core` 动态库

目标：保留代码目录和类职责分层，只生成一个核心库。

修改：

```text
src/frontier_strategy_core/CMakeLists.txt
src/frontier_strategy_core/cmake/frontier_strategy_coreConfig.cmake.in
src/frontier_strategy_core/test/downstream_smoke/
```

目标：

```cmake
add_library(frontier_strategy_core SHARED
  src/detector/frontier_detector.cpp
  src/scoring/...
  src/selector/...
  src/policy/frontier_strategy_policy.cpp
)

add_library(
  frontier_strategy_core::frontier_strategy_core
  ALIAS frontier_strategy_core
)
```

删除 detection/scoring/selection/policy 四个公开动态库 target。所有单元测试统一链接 `frontier_strategy_core`，但测试文件仍按 detector、scoring、selection、policy 分开。

验收：

- 新 install prefix 下只产生一个 `libfrontier_strategy_core.so`；
- 下游通过 namespaced target `frontier_strategy_core::frontier_strategy_core` 构建；
- detector/scoring/selection/policy 测试全部通过；
- 不改变评分、筛选、黑名单、fallback 和可达性行为。

检查 `.so` 时必须使用新的 build/install base，不能让旧安装目录的残留库造成误判。

建议 commit：

```text
refactor: consolidate frontier strategy core library
```

---

## 5. 明确禁止的范围扩张

本轮禁止：

- 实现 `LearnedFrontierRanker`；
- 引入 ONNX、Torch、TensorRT 或模型配置；
- 实现运行时插件加载系统；
- 扩大 `ExplorationObservation` 以包装 ROS/Nav2 回调；
- 修改 Frontier 算法权重和选点规则；
- 修改 BT XML 行为逻辑；
- 修改 ROS topic/service/action 名称；
- 修改地图保存协议或增加 map I/O node；
- 修改 TaskManager 状态机；
- 顺手处理 `issue.md` 中的性能和局部最优问题；
- 提交 `.gitignore`、`issue.md`、`development/`、`.skillmatrix/`、build/install/log 产物。

---

## 6. 测试策略

### 6.1 Windows 本地静态检查

Windows 只作为源码编辑和静态检查环境，不作为 ROS 构建通过证据。

```powershell
git diff --check
rg "IExplorationPolicy|ExplorationObservation|MapSnapshot|exploration_nodes_common" src
rg "ament|rclcpp|nav_msgs|geometry_msgs|nav2" src/navigation_core
```

### 6.2 Ubuntu 干净构建

通过项目配置的远程执行通道，将本轮涉及的源码目录精确同步到：

```text
<workspace>
```

同步前先执行 `git status --short`，不得覆盖远端不属于本轮的修改。构建使用独立目录，避免旧 target 和旧 `.so` 残留：

```bash
cd <workspace>
source /opt/ros/humble/setup.bash

colcon --log-base log_arch_fix build \
  --build-base build_arch_fix \
  --install-base install_arch_fix \
  --packages-up-to exploration_nodes map_lifecycle \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### 6.3 Ubuntu 模块测试

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install_arch_fix/setup.bash

colcon --log-base log_arch_fix_test test \
  --build-base build_arch_fix \
  --install-base install_arch_fix \
  --packages-select \
    exploration_core \
    grid_map_core \
    robot_geometry_core \
    navigation_core \
    frontier_strategy_core \
    frontier_strategy_ros \
    exploration_nodes \
    map_lifecycle \
  --event-handlers console_direct+

colcon test-result --test-result-base build_arch_fix --verbose
```

如果 `colcon test` 因 Ubuntu 缺少 `ament_cmake_test` Python 模块而在外层失败，必须同时记录环境错误，并直接执行标准 CTest/测试二进制确认纯 Core 结果；不能把环境错误记成代码回归，也不能因此省略 ROS package build。

### 6.4 Navigation 独立 CMake 验证

在 `grid_map_core` 和 `robot_geometry_core` 已安装后执行：

```bash
cmake -S src/navigation_core -B /tmp/navigation_core_standalone \
  -DCMAKE_PREFIX_PATH="<workspace>/install_arch_fix/grid_map_core;<workspace>/install_arch_fix/robot_geometry_core" \
  -DBUILD_TESTING=ON

cmake --build /tmp/navigation_core_standalone
ctest --test-dir /tmp/navigation_core_standalone --output-on-failure
```

还要运行一个最小 downstream consumer，验证安装后的：

```cmake
find_package(navigation_core CONFIG REQUIRED)
target_link_libraries(smoke PRIVATE navigation_core::navigation_core)
```

### 6.5 Frontier 安装产物检查

```bash
find <workspace>/install_arch_fix/frontier_strategy_core/lib \
  -maxdepth 1 -type f -name 'libfrontier_strategy*.so*' -print
```

只应看到 `libfrontier_strategy_core.so` 及其合法版本软链接；不应出现 detection/scoring/selection/policy 四个独立库。

### 6.6 集成测试责任边界

Luna 负责：

- Ubuntu 编译；
- Core 单元测试；
- ROS adapter/component 测试；
- 下游 CMake package contract smoke。

用户负责最终 Gazebo 集成测试：

```text
Map → FrontierStrategy → Candidate → Navigation → Outcome → MapLifecycle
```

Luna 交付时应给出完整启动命令和预期状态，但不要声称未运行的 Gazebo 测试已经通过。

---

## 7. 最终验收清单

全部满足才能认为本轮修复完成：

- [ ] 生产代码不再声明或使用 `IExplorationPolicy`；
- [ ] 生产路径可注入 `IFrontierRanker`；
- [ ] 默认行为仍使用 `RuleBasedFrontierRanker`；
- [ ] Fake Ranker 测试证明注入进入真实候选计算链；
- [ ] 不存在 `exploration_nodes_common`；
- [ ] 两个 Node executable 只链接各自实现 target；
- [ ] `navigation_core` 的源码、CMake、package.xml 均无 ament/ROS 依赖；
- [ ] 标准 CMake 下游能 `find_package(navigation_core)`；
- [ ] 不存在 `MapSnapshot`，Map Lifecycle 使用 `grid_map_core::GridMap`；
- [ ] ROS 地图转换保留 origin；
- [ ] Frontier Strategy Core 只安装一个核心 `.so`；
- [ ] Ubuntu ROS2 构建通过；
- [ ] 受影响模块测试通过；
- [ ] ROS topic/service/action 和状态机语义未改变；
- [ ] `.gitignore`、`issue.md` 和工程过程目录未进入提交；
- [ ] 每个阶段独立 commit；
- [ ] 未经用户明确要求不 push。

---

## 8. Luna 交付说明格式

Luna 完成后必须按以下结构汇报：

```text
1. 每条评审问题如何处理
2. 修改过的生产文件
3. 新增或修改的测试
4. Windows 静态检查结果
5. Ubuntu 构建命令和退出结果
6. Ubuntu 测试命令、通过数和失败数
7. 未运行的集成测试
8. 保留的本地无关修改
9. commit 列表
10. 是否 push（默认否）
```
