#pragma once

#include <cstdint>
#include <string>

namespace exploration_learning::schema
{

/// @brief 单个候选 frontier 的训练特征结构说明。
///
/// 当前 MVP 先通过 JSONL 写入，不直接实例化该结构体；保留 schema 头文件用于约束字段含义。
struct CandidateRecord
{
  /// @brief 候选编号，同一 decision 内唯一。
  std::string candidate_id;

  /// @brief 规则 baseline 输出的总分。
  double score_total{0.0};

  /// @brief 候选到机器人当前位置的距离，单位米。
  double distance_to_robot{0.0};

  /// @brief 候选 clearance，单位米。
  double clearance{0.0};

  /// @brief 候选周围 unknown cell 比例。
  double unknown_ratio{0.0};

  /// @brief planner 返回路径长度，单位米。
  double path_length{0.0};

  /// @brief 候选是否通过可达性检查。
  bool reachable{true};

  /// @brief 候选是否为最终选中目标。
  bool selected{false};

  /// @brief 候选被拒绝或未选中的原因。
  std::string reject_reason;
};

/// @brief 一次 frontier 决策样本结构说明。
struct DecisionRecord
{
  /// @brief episode 标识。
  std::string episode_id;

  /// @brief 当前 episode 内递增的决策编号。
  std::uint64_t decision_id{0U};

  /// @brief 决策事件时间戳，单位秒。
  double timestamp_sec{0.0};

  /// @brief 最终选中候选编号；未知时为空。
  std::string selected_candidate_id;
};

}  // namespace exploration_learning::schema
