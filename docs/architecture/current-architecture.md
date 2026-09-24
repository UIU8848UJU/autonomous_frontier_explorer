# 当前架构总览

> 状态：持续维护
>
> 同步基线：`d7be6c6`（2026-09-24）
>
> 说明：本文描述当前模块职责；具体接口和依赖以代码及测试为准。

## Pure Core
```mermaid
flowchart LR
    GEO["robot_geometry_core<br/>二维几何"]

    MAP["grid_map_core<br/>栅格地图模型"]

    EXP["exploration_core<br/>探索领域协议"]

    NAV["navigation_core<br/>路径/目标安全"]

    FRONTIER["frontier_strategy_core<br/>Frontier 策略"]

    GEO --> NAV
    MAP --> NAV

    GEO --> FRONTIER
    MAP --> FRONTIER
    EXP --> FRONTIER
```

# Frontier：去哪儿

```mermaid
flowchart TD
    MAP["/map"]
    COST["/global_costmap/costmap"]
    TF["TF2<br/>机器人位置"]

    NODE["FrontierStrategyNode"]

    ADAPTER["grid_map_ros<br/>ROS Map → GridMap"]

    PROVIDER["FrontierGoalProvider<br/>frontier_strategy_ros"]

    POLICY["FrontierStrategyPolicy"]

    DET["FrontierDetector"]
    PRUNE["FrontierPruner"]
    RANK["FrontierRanker"]

    OUT["FrontierCandidate[]"]

    MAP --> NODE
    COST --> NODE
    TF --> NODE

    NODE --> ADAPTER
    ADAPTER --> PROVIDER

    PROVIDER --> POLICY

    POLICY --> DET
    DET --> PRUNE
    PRUNE --> RANK

    RANK --> OUT
```

## 如何找的

```text
/map + /global_costmap/costmap + robot pose
        ↓
FrontierDetector
  找到“已知自由区 + 邻接未知区”
        ↓
8 邻域聚类
        ↓
FrontierPruner
  生成候选点并执行硬门禁
        ↓
RuleBasedFrontierRanker
(这里有问题，信息熵最开始其实可以大一点，后续应该以为准做一个动态)
  距离、簇大小、重试、未知比例、信息增益打分
        ↓
GetFrontierCandidates 返回候选列表
        ↓
BT SelectFeasibleFrontier
  调用 NavigationNode 检查可达性
（可达性的依据是什么是否是太严格了?）
        ↓
NavigateToFrontier
        ↓
导航过程中 Prefetch 下一轮候选
        ↓
导航成功后重新选择
```


# Navigation：能不能去 + 怎么去
```mermaid
flowchart TD
    GOAL["目标 Pose"]

    NN["NavigationNode"]

    NC["navigation_core"]

    FOOT["FootprintGoalValidator"]
    PATH["PathSafetyChecker"]
    GATE["SingleGoalGate"]

    PLANNER["Nav2 Planner<br/>ComputePathToPose"]

    NAVIGATOR["Nav2 Navigator<br/>NavigateToPose"]

    COSTMAP["Global Costmap"]

    ROBOT["底盘 / Robot"]

    GOAL --> NN

    NN --> NC

    NC --> FOOT
    NC --> PATH
    NC --> GATE

    NN --> PLANNER
    COSTMAP --> NN

    PLANNER --> PATH

    NN --> NAVIGATOR
    NAVIGATOR --> ROBOT
```



# BT：把“去哪”和“怎么去”组合起来

```mermaid
flowchart TD
    START["开始一次探索循环"]

    GET["ComputeFrontierCandidates<br/>获取候选"]

    DONE{"探索完成？"}

    SELECT["SelectFeasibleFrontier<br/>检查候选可执行性"]

    FOUND{"找到可执行目标？"}

    PAR["Parallel"]

    NAV["NavigateToFrontier"]

    PREFETCH["PrefetchFrontierCandidates<br/>预取下一轮"]

    SUCCESS{"导航成功？"}

    FAIL["MarkFrontierFailed"]

    START --> GET

    GET --> DONE

    DONE -->|是| END["Exploration Complete"]

    DONE -->|否| SELECT

    SELECT --> FOUND

    FOUND -->|否| FAIL

    FOUND -->|是| PAR

    PAR --> NAV
    PAR --> PREFETCH

    NAV --> SUCCESS

    SUCCESS -->|成功| GET
    SUCCESS -->|失败| FAIL

    FAIL --> GET
```


# TaskManager：机器人现在到底在干嘛

```mermaid
flowchart TD
    USER["用户 / 上层任务"]

    TM["TaskManagerNode"]

    MAP["Mapping Task"]
    NAV["Navigation Task"]
    STOP["Stop All"]

    BT["ExplorationBtOrchestrator"]

    ML["MapLifecycle"]

    SLAM["SLAM Toolbox"]
    NAV2["Nav2"]

    USER --> TM

    TM --> MAP
    TM --> NAV
    TM --> STOP

    MAP --> SLAM
    MAP --> BT

    NAV --> NAV2

    BT --> ML
```


TaskManager
├── Exploration
├── ReturnHome
├── Patrol
├── Mapping
├── NavigateToPoint
└── Docking


# Learning：旁路观察系统
```mermaid
flowchart LR
    MAP["/map"]

    DEC["Frontier Decision"]

    NAV["Navigation Result"]

    STATE["Exploration State"]

    REC["DatasetRecorderNode"]

    BUF["EventBuffer"]

    PLUGIN["FrontierDecisionPlugin"]

    WRITER["DatasetWriter"]

    DATA["Episode Dataset"]

    MAP --> REC
    DEC --> REC
    NAV --> REC
    STATE --> REC

    REC --> BUF

    BUF --> PLUGIN

    PLUGIN --> WRITER

    WRITER --> DATA
```


# Bringup：整个系统怎么活起来

```mermaid
flowchart TD
    LAUNCH["full_system.launch.py"]

    NAV2["Nav2"]
    SLAM["SLAM / Localization"]
    FRONTIER["FrontierStrategyNode"]
    NAV["NavigationNode"]

    GATE["ReadinessGate"]

    BT["ExplorationBtOrchestrator"]
    TM["TaskManager"]
    ML["MapLifecycle"]

    LAUNCH --> NAV2
    LAUNCH --> SLAM
    LAUNCH --> FRONTIER
    LAUNCH --> NAV

    NAV2 --> GATE
    SLAM --> GATE
    FRONTIER --> GATE
    NAV --> GATE

    GATE -->|"READY"| BT
    GATE -->|"READY"| TM
    GATE -->|"READY"| ML
```


# 最后才汇总成宏观架构
```mermaid
flowchart TB

    TASK["Task Layer<br/>TaskManager"]

    BT["Behavior Layer<br/>BehaviorTree"]

    subgraph CAP["Capability Layer"]
        FRONTIER["Frontier Strategy<br/>去哪"]
        NAV["Navigation<br/>怎么去"]
        MAPLIFE["Map Lifecycle<br/>地图生命周期"]
    end

    subgraph ADAPTER["ROS Adapter Layer"]
        FSROS["frontier_strategy_ros"]
        GRIDROS["grid_map_ros"]
    end

    subgraph CORE["Pure Domain Core"]
        EXP["exploration_core"]
        FSC["frontier_strategy_core"]
        NAVC["navigation_core"]
        GRID["grid_map_core"]
        GEO["robot_geometry_core"]
    end

    INFRA["Robot Infrastructure<br/>ROS2 / Nav2 / SLAM / TF / Costmap / Chassis"]

    LEARN["Learning Sidecar<br/>Dataset / Training"]

    TASK --> BT

    BT --> FRONTIER
    BT --> NAV

    TASK --> MAPLIFE

    FRONTIER --> FSROS
    FSROS --> FSC

    FRONTIER --> GRIDROS
    NAV --> GRIDROS

    GRIDROS --> GRID

    FSC --> EXP
    FSC --> GRID
    FSC --> GEO

    NAV --> NAVC
    NAVC --> GRID
    NAVC --> GEO

    INFRA --> FRONTIER
    INFRA --> NAV
    INFRA --> MAPLIFE

    FRONTIER -.数据.-> LEARN
    NAV -.数据.-> LEARN
    BT -.状态.-> LEARN
    INFRA -.地图.-> LEARN
```


# 总架构图
```mermaid
flowchart TB

    %% =========================
    %% 启动 / 任务层
    %% =========================
    subgraph L0["① 系统启动 / 任务管理层"]
        BRINGUP["autonomousr_explorer_bringup<br/>Launch + ReadinessGate"]
        TM["task_manager<br/>TaskManagerNode"]
        ML["map_lifecycle<br/>MapLifecycleNode"]
    end

    %% =========================
    %% BT orchestration
    %% =========================
    subgraph L1["② 探索流程编排层"]
        BTO["exploration_bt<br/>ExplorationBtOrchestratorNode"]

        BT["BehaviorTree.CPP<br/>exploration_tree.xml"]

        CFC["ComputeFrontierCandidates"]
        SFF["SelectFeasibleFrontier"]

        PAR["Parallel"]

        NAVF["NavigateToFrontier"]
        PREFETCH["PrefetchFrontierCandidates"]

        FAIL["MarkFrontierFailed"]
        COMPLETE["IsExplorationComplete"]

        BTO --> BT
        BT --> CFC
        BT --> SFF
        BT --> PAR
        PAR --> NAVF
        PAR --> PREFETCH
        BT --> FAIL
        BT --> COMPLETE
    end

    %% =========================
    %% ROS capability nodes
    %% =========================
    subgraph L2["③ ROS 能力节点层"]
        FSN["FrontierStrategyNode<br/>exploration_nodes"]
        NN["NavigationNode<br/>exploration_nodes"]

        RI["robot_interfaces<br/>msg / srv / action"]
    end

    %% =========================
    %% ROS adapters
    %% =========================
    subgraph L3["④ ROS Adapter 层"]
        FGP["FrontierGoalProvider<br/>frontier_strategy_ros"]

        GMR["grid_map_ros<br/>OccupancyGrid / Costmap Adapter"]
    end

    %% =========================
    %% Pure core
    %% =========================
    subgraph L4["⑤ ROS-Free Pure C++ Core"]
        FSC["frontier_strategy_core<br/>FrontierStrategyPolicy"]

        DET["FrontierDetector"]
        PRUNE["FrontierPruner"]
        RANK["RuleBasedFrontierRanker"]

        EC["exploration_core<br/>探索协议 / 策略抽象"]

        GMC["grid_map_core<br/>GridMap 领域模型"]

        GEO["robot_geometry_core<br/>2D Geometry"]

        NC["navigation_core<br/>Footprint Validator<br/>Path Safety Checker<br/>Single Goal Gate"]

        FSC --> DET
        FSC --> PRUNE
        FSC --> RANK

        FSC --> EC
        FSC --> GMC
        FSC --> GEO
    end

    %% =========================
    %% ROS / middleware infra
    %% =========================
    subgraph INFRA["ROS / Navigation / Robot 基础设施"]
        SLAM["SLAM Toolbox<br/>/map"]

        NAV2["Nav2<br/>Planner / Navigator"]

        COSTMAP["Nav2 Global Costmap<br/>/global_costmap/costmap"]

        TF["TF2<br/>map ← base_link"]

        RVIZ["RViz"]

        CHASSIS["chassis_bridge<br/>cmd_vel / odom / chassis"]
    end

    %% =========================
    %% Learning sidecar
    %% =========================
    subgraph LEARN["⑥ Learning / 数据旁路"]
        DR["exploration_learning<br/>DatasetRecorderNode"]

        EB["EventBuffer"]

        PF["DataRecordPluginFactory"]

        FDP["FrontierDecisionPlugin"]

        DW["DatasetWriter"]

        DATA["episode_metadata.json<br/>decision_records.jsonl"]

        DR --> EB
        EB --> PF
        PF --> FDP
        FDP --> DW
        DW --> DATA
    end


    %% =========================
    %% startup relationship
    %% =========================
    BRINGUP --> SLAM
    BRINGUP --> NAV2
    BRINGUP --> FSN
    BRINGUP --> NN

    BRINGUP -->|"Readiness Gate<br/>ready 后启动"| BTO
    BRINGUP --> TM
    BRINGUP --> ML

    TM -->|"start / stop exploration"| BTO


    %% =========================
    %% BT -> capability
    %% =========================
    CFC -->|"GetFrontierCandidates"| FSN

    PREFETCH -->|"异步预取候选"| FSN

    FAIL -->|"MarkFrontierFailed"| FSN

    SFF -->|"CheckGoalFeasibility"| NN

    NAVF -->|"NavigateToPose"| NN


    %% =========================
    %% Frontier capability
    %% =========================
    FSN --> FGP
    FGP --> FSC

    SLAM -->|"/map"| FSN
    TF --> FSN

    FSN --> GMR
    GMR --> GMC

    FSN -->|"marker / state"| RVIZ


    %% =========================
    %% Navigation capability
    %% =========================
    NN --> NC

    NN -->|"ComputePathToPose"| NAV2
    NN -->|"NavigateToPose"| NAV2

    COSTMAP --> GMR
    GMR --> NN

    NAV2 --> COSTMAP

    NAV2 -->|"/cmd_vel"| CHASSIS
    CHASSIS -->|"odom / TF"| NAV2


    %% =========================
    %% interface boundary
    %% =========================
    RI -.定义通信协议.-> FSN
    RI -.定义通信协议.-> NN
    RI -.定义通信协议.-> BTO


    %% =========================
    %% state
    %% =========================
    BTO -->|"/exploration_state"| TM
    BTO -->|"/exploration_state"| ML


    %% =========================
    %% learning sidecar
    %% =========================
    SLAM -. "/map" .-> DR

    FSN -. "decision debug" .-> DR

    NN -. "navigation result debug" .-> DR

    BTO -. "/exploration_state" .-> DR
```
