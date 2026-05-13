#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "frontier_explorer_core/costmap/costmap_adapter.hpp"
#include "frontier_explorer_core/selector/candidates/frontier_decision_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "frontier_explorer_core/types/frontier_types.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace frontier_explorer
{

/// @brief: Frontier RViz marker 发布器，集中负责 raw frontier、候选、评分、目标和黑名单的可视化
class FrontierMarkerPublisher
{
public:
    /// @brief: 构造 marker 发布器
    /// @param node ROS2 节点指针，用于创建 publisher 和获取时间
    /// @param logger ROS2 日志器
    /// @param frame_id marker 使用的坐标系
    explicit FrontierMarkerPublisher(
        rclcpp::Node * node,
        const rclcpp::Logger & logger,
        const std::string & frame_id = "map");

    /// @brief: 清空所有 frontier marker topic 上的旧 marker
    void clearAll() const;

    /// @brief: 清空候选点、评分文本和当前选中目标 marker
    void clearCandidateMarkers() const;

    /// @brief: 清空被拒绝 frontier marker
    void clearRejectedMarkers() const;

    /// @brief: 发布原始 frontier cluster 点云 marker
    /// @param clusters detector 输出的 frontier clusters
    /// @param costmap 用于把 grid cell 转换到世界坐标
    void publishRawFrontiers(
        const std::vector<FrontierCluster> & clusters,
        const CostmapAdapter & costmap) const;

    /// @brief: 发布 pruner 产生的候选点 marker
    /// @param candidates frontier 候选点列表
    /// @param costmap 用于把 grid cell 转换到世界坐标
    /// @param selected_goal 本轮最终选中的 goal，用于颜色区分
    void publishCandidates(
        const std::vector<FrontierCandidate> & candidates,
        const CostmapAdapter & costmap,
        const std::optional<GridCell> & selected_goal = std::nullopt) const;

    /// @brief: 发布被拒绝候选点 marker，当前预留接口
    /// @param rejected_candidates 被拒绝候选点列表
    /// @param costmap 用于把 grid cell 转换到世界坐标
    void publishRejectedCandidates(
        const std::vector<FrontierCandidate> & rejected_candidates,
        const CostmapAdapter & costmap) const;

    /// @brief: 发布被拒绝的原始 frontier cluster 点云 marker
    /// @param clusters 被拒绝的 frontier clusters
    /// @param costmap 用于把 grid cell 转换到世界坐标
    void publishRejectedFrontiers(
        const std::vector<FrontierCluster> & clusters,
        const CostmapAdapter & costmap) const;

    /// @brief: 发布带评分文本的候选点 marker
    /// @param scored_candidates 已评分候选点列表
    /// @param costmap 用于把 grid cell 转换到世界坐标
    /// @param selected_goal 本轮最终选中的 goal，用于颜色区分
    void publishScoredCandidates(
        const std::vector<ScoredFrontierCandidate> & scored_candidates,
        const CostmapAdapter & costmap,
        const std::optional<GridCell> & selected_goal = std::nullopt) const;

    /// @brief: 发布最终选中的目标 marker
    /// @param goal 选中的目标栅格
    /// @param robot 机器人当前栅格
    /// @param costmap 用于把 grid cell 转换到世界坐标
    void publishSelectedGoal(
        const GridCell & goal,
        const GridCell & robot,
        const CostmapAdapter & costmap) const;

    /// @brief: 发布黑名单目标 marker
    /// @param blacklist 黑名单目标栅格列表
    /// @param costmap 用于把 grid cell 转换到世界坐标
    void publishBlacklist(
        const std::vector<GridCell> & blacklist,
        const CostmapAdapter & costmap) const;

private:
    void publishDeleteAll(
        const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher) const;

    visualization_msgs::msg::Marker makeDeleteAllMarker() const;

    visualization_msgs::msg::Marker makeDeleteMarker(
        const std::string & marker_namespace,
        int id) const;

    void appendStaleDeleteMarkers(
        visualization_msgs::msg::MarkerArray & array,
        const std::string & marker_namespace,
        std::size_t current_count,
        std::size_t previous_count) const;

    visualization_msgs::msg::Marker makeBaseMarker(
        const std::string & marker_namespace,
        int id,
        int type) const;

    bool cellToWorldPoint(
        const GridCell & cell,
        const CostmapAdapter & costmap,
        geometry_msgs::msg::Point & point) const;

private:
    rclcpp::Node * node_{nullptr};
    rclcpp::Logger logger_;
    std::string frame_id_{"map"};

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr raw_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rejected_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scored_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr selected_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr blacklist_pub_;

    mutable std::size_t raw_marker_count_{0U};
    mutable std::size_t candidate_marker_count_{0U};
    mutable std::size_t rejected_candidate_marker_count_{0U};
    mutable std::size_t rejected_frontier_marker_count_{0U};
    mutable std::size_t scored_marker_count_{0U};
    mutable std::size_t selected_marker_count_{0U};
    mutable std::size_t blacklist_marker_count_{0U};
};

}  // namespace frontier_explorer
