# 项目文档

本目录保存能够跨开发机器复用的架构说明、重构路线和历史实施记录。主机地址、绝对路径、临时任务日志和原始测试输出不进入仓库。

当前代码基线：`d7be6c6`（2026-09-24）。

## 当前架构

- [当前架构总览](architecture/current-architecture.md)：核心库、ROS 适配层、BT、任务编排和启动层的职责关系。
- [PC、Jetson 与嵌入式边界](architecture/deployment-boundaries.md)：不同算力平台的部署职责和数据流。

## 持续路线图

- [多核心架构路线图](refactoring/roadmap/multi-core-roadmap.md)：地图、探索、导航和训练系统的后续演进方向。
- [探索模块重构路线图](refactoring/roadmap/exploration-refactoring-roadmap.md)：探索行为、Frontier 策略、导航和训练闭环的目标边界。

## 历史记录

以下文档用于解释设计背景，不代表当前代码仍存在其中列出的全部问题：

- [探索重构测试策略归档](../strategy/README.md)
- [多核心架构重构与训练系统建设计划](refactoring/archive/2026-09-04-multi-core-architecture-plan.md)
- [探索架构收敛修复计划](refactoring/archive/2026-09-12-exploration-architecture-fix-plan.md)
- [探索收尾与选点性能分析](refactoring/archive/2026-09-12-exploration-tail-analysis.md)
- [探索收尾与连续选点实施计划](refactoring/archive/2026-09-12-exploration-tail-implementation-plan.md)

## 维护规则

- 当前行为以代码、测试和最新提交为准。
- 历史计划只追加结果说明，不反向改写当时的分析结论。
- 新文档使用仓库相对路径，不记录主机别名、用户目录和工作区绝对路径。
- 可复现命令保留占位符，例如 `<workspace>`，避免与单台开发机绑定。
