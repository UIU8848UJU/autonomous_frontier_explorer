#pragma once

#include <cstdint>
#include <string>

namespace exploration_learning
{

/// @brief 单次仿真或真实采集 episode 的元数据。
///
/// 该结构体用于记录 episode 级别的上下文，便于后续把 records.jsonl
/// 与地图、随机种子、初始位姿和参数文件关联起来。
struct EpisodeMetadata
{
  /// @brief 当前 episode 的唯一标识。
  std::string episode_id;

  /// @brief 当前 episode 使用的仿真世界或地图名称。
  std::string world_name;

  /// @brief 当前 episode 使用的随机种子。
  std::uint64_t seed{0U};

  /// @brief 机器人初始位姿的 x 坐标，单位米。
  double spawn_x{0.0};

  /// @brief 机器人初始位姿的 y 坐标，单位米。
  double spawn_y{0.0};

  /// @brief 机器人初始 yaw 角，单位弧度。
  double spawn_yaw{0.0};

  /// @brief 当前 episode 使用的参数文件路径。
  std::string params_file;
};

}  // namespace exploration_learning
