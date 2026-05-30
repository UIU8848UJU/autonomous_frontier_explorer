#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "exploration_learning/collector/data_record_plugin.hpp"

namespace exploration_learning::plugins
{

/// @brief Frontier decision 数据采集插件配置。
struct FrontierDecisionPluginConfig
{
  /// @brief 当前 episode 标识。
  std::string episode_id;

  /// @brief frontier 决策调试 JSON topic。
  std::string decision_topic;

  /// @brief 导航结果调试 JSON topic。
  std::string navigation_result_topic;

  /// @brief 地图摘要事件的内部 topic 名称。
  std::string map_summary_topic;

  /// @brief 探索状态事件的内部 topic 名称。
  std::string exploration_state_topic;
};

/// @brief 将 frontier decision debug event 转换为训练用 decision record。
///
/// 第一版不解析上游 JSON 的内部字段，而是把 frontier decision、最近地图摘要、
/// 最近导航结果和探索状态作为上下文稳定写入，后续可在该插件内替换为强 schema 解析。
class FrontierDecisionPlugin : public collector::IDataRecordPlugin
{
public:
  /// @brief 返回插件注册名。
  /// @return 插件注册名。
  static std::string plugin_name();

  /// @brief 构造 frontier decision 插件。
  /// @param config 插件运行配置。
  explicit FrontierDecisionPlugin(const FrontierDecisionPluginConfig & config);

  /// @brief 返回插件名称。
  /// @return frontier_decision。
  std::string name() const override;

  /// @brief 处理 topic event 并在 frontier decision 事件到达时生成 record。
  /// @param event 当前事件。
  /// @param buffer 事件缓存。
  /// @return decision record JSON 列表。
  std::vector<std::string> handle_event(
    const collector::TopicEvent & event,
    const collector::EventBuffer & buffer) override;

private:
  /// @brief 最近一次 frontier candidates 中的候选快照。
  struct CandidateSnapshot
  {
    std::string candidate_id;
    double x{0.0};
    double y{0.0};
    double score_total{0.0};
    double distance_to_robot{0.0};
    double clearance{0.0};
    double unknown_ratio{0.0};
    double path_length{0.0};
    bool path_length_valid{false};
    bool reachable{true};
    std::string source_json;
  };

  /// @brief 从缓存中读取最近上下文并拼装 JSON record。
  /// @param event 当前 frontier decision 事件。
  /// @param buffer 事件缓存。
  /// @return 完整 decision record JSON object 字符串。
  std::string build_decision_record(
    const collector::TopicEvent & event,
    const collector::EventBuffer & buffer,
    const std::string & selected_candidate_id,
    bool has_outcome,
    const std::string & decision_context_json,
    const std::string & outcome_context_json);

  /// @brief 处理 frontier decision topic 事件。
  /// @param event 当前事件。
  /// @param buffer 事件缓存。
  /// @return record JSON 列表。
  std::vector<std::string> handle_decision_event(
    const collector::TopicEvent & event,
    const collector::EventBuffer & buffer);

  /// @brief 处理 navigation debug topic 事件。
  /// @param event 当前事件。
  /// @param buffer 事件缓存。
  /// @return record JSON 列表。
  std::vector<std::string> handle_navigation_event(
    const collector::TopicEvent & event,
    const collector::EventBuffer & buffer);

  /// @brief 从 frontier_candidates raw JSON 中更新候选缓存。
  /// @param raw_json 上游 frontier decision raw JSON。
  void update_candidates_from_frontier_json(const std::string & raw_json);

  /// @brief 用 navigation / feasibility event 中的路径结果更新候选缓存。
  /// @param raw_json 上游 navigation raw JSON。
  void update_candidate_from_navigation_json(const std::string & raw_json);

  /// @brief 根据 goal 坐标匹配最近候选。
  /// @param raw_json 含 selected 或 goal 字段的 raw JSON。
  /// @return 匹配到的 candidate id。
  std::string match_selected_candidate_id(const std::string & raw_json) const;

  /// @brief 解析 wrapper 中的 raw_json 字段。
  /// @param payload_json TopicEvent payload。
  /// @return raw_json 字符串；不存在时返回 std::nullopt。
  std::optional<std::string> extract_raw_json(const std::string & payload_json) const;

  /// @brief 将当前候选缓存转换为 JSON array。
  /// @param selected_candidate_id 被选中候选 id。
  /// @return candidates JSON array。
  std::string candidates_to_json(const std::string & selected_candidate_id) const;

  /// @brief 插件配置。
  FrontierDecisionPluginConfig config_;

  /// @brief 当前 episode 内递增的 decision id。
  std::uint64_t next_decision_id_{0U};

  /// @brief 最近一次候选列表缓存。
  std::vector<CandidateSnapshot> latest_candidates_;

  /// @brief 最近一次候选生成事件 payload，用作闭环样本的 decision_context。
  std::string latest_decision_context_json_{"null"};
};

}  // namespace exploration_learning::plugins
