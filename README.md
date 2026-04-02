

# 环境版本（Environment）

- OS: Ubuntu 22.04
- ROS: ROS 2 Humble
- Navigation: Nav2
- Simulator: Gazebo
- Language: C++/14
- Build tool: colcon

# 启动（Run it）

**构建（build）**

```bash
cd ~/mk_nav2   # 你的项目目录 your workspace
colcon build --packages-select task_manager frontier_explorer
```

**启动（Run）**
```bash
cd ~/mk_nav2 # 你的项目目录 your workspace
source install/setup.bash
ros2 launch task_manager sim_slam_bringup.launch.py # 启动
```

# 效果

![result](image/result.png)

# 目前问题


## Frontier_Expolorer
1）只选最近 frontier

策略比较朴素。可能导致机器人来回挑局部最近点，不一定全局效率最好。

2）没有处理 goal feedback

这里只有 goal_response_callback 和 result_callback，没有 feedback callback。
所以拿不到实时剩余距离、当前进度。

3）没有失败恢复策略

机器人翻了之后没有能力继续建图了，后续的数据都不可控了

比如某个 frontier 总是到不了，这里没有黑名单机制，也没有重试次数限制。

4）centroid 不一定真可达

虽然做了安全检查，但 centroid 只是几何中心，不代表路径规划一定能到。

5）遇到障碍物的时候会在障碍物旁边等很久，计划last_goal_grid_。



