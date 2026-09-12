#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "frontier_strategy_core/reachability/frontier_reachability_result.hpp"
#include "frontier_strategy_core/scoring/frontier_scorer.hpp"
#include "frontier_strategy_core/scoring/frontier_ranker.hpp"
#include "frontier_strategy_core/scoring/frontier_scoring_weights.hpp"
#include "frontier_strategy_core/selector/filters/frontier_pruner.hpp"
#include "frontier_strategy_core/selector/state/frontier_selection_state.hpp"

namespace frontier_strategy
{

struct FrontierSelectionCoreConfig
{
    int max_retry_count{2};
    int max_cluster_retry_count{3};
    bool defer_small_clusters{true};
    std::size_t small_cluster_size_threshold{3U};
    bool require_reachable_goal{false};
};

/// @brief 不依赖 ROS 的选择状态机和候选池策略。
class FrontierSelectionPolicy
{
public:
    using ReachabilityCheck =
        std::function<FrontierReachabilityResult(FrontierCandidate &)>;

    FrontierSelectionPolicy(
        FrontierSelectionCoreConfig config = FrontierSelectionCoreConfig{},
        FrontierScoringWeights scoring_weights = FrontierScoringWeights{},
        std::shared_ptr<IFrontierRanker> ranker = {});

    /// @brief 开始新的探索任务，清理上一次任务留下的选择状态。
    void reset();

    FrontierPruningContext pruning_context() const;
    void record_failed_clusters(const std::vector<GridCell> & cluster_ids);

    std::optional<ScoredFrontierCandidate> choose_best_candidate(
        const std::vector<FrontierCandidate> & candidates,
        const ReachabilityCheck & reachability_check = {});
    std::vector<ScoredFrontierCandidate> rank_candidates(
        const std::vector<FrontierCandidate> & candidates,
        const ReachabilityCheck & reachability_check = {});

    void set_last_goal(const GridCell & goal);
    void mark_goal_failed(const GridCell & goal);
    void mark_goal_succeeded(const GridCell & goal);

    const std::vector<ScoredFrontierCandidate> & last_scored_candidates() const;
    std::vector<GridCell> blacklisted_goals() const;
    int retry_count_for_goal(const GridCell & goal) const;
    bool is_goal_blacklisted(const GridCell & goal) const;
    std::size_t clear_blacklist();

private:
    struct CandidatePools
    {
        std::vector<FrontierCandidate> normal;
        std::vector<FrontierCandidate> small;
    };

    CandidatePools partition_candidates(
        const std::vector<FrontierCandidate> & candidates) const;
    std::vector<ScoredFrontierCandidate> score_and_rank_candidates(
        const std::vector<FrontierCandidate> & candidates,
        const ReachabilityCheck & reachability_check) const;
    std::optional<ScoredFrontierCandidate> choose_best_scored_candidate(
        const std::vector<FrontierCandidate> & candidates,
        const ReachabilityCheck & reachability_check);
    void mark_cluster_failed(const GridCell & cluster_id);
    void mark_cluster_succeeded(const GridCell & cluster_id);

    FrontierSelectionCoreConfig config_;
    std::shared_ptr<IFrontierRanker> ranker_;
    FrontierSelectionState state_;
    std::vector<ScoredFrontierCandidate> last_scored_candidates_;
};

}  // namespace frontier_strategy
