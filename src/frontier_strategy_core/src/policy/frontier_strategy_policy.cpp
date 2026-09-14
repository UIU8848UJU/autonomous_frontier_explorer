#include "frontier_strategy_core/policy/frontier_strategy_policy.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

namespace frontier_strategy
{

FrontierStrategyPolicy::FrontierStrategyPolicy(
    FrontierStrategyPolicyConfig config,
    std::shared_ptr<IFrontierRanker> ranker)
: config_(std::move(config)),
  detector_(config_.obstacle_search_radius_cells),
  pruner_(
      config_.min_goal_distance_m,
      config_.max_retry_count,
      config_.max_cluster_retry_count,
      config_.min_cluster_size,
      config_.unknown_margin_cells,
      config_.goal_inset_cells,
      config_.max_unknown_ratio,
      config_.viewpoint_retreat_distances_m,
      config_.viewpoint_sample_radii_m,
      config_.viewpoint_angle_step_deg,
      config_.sensor_range_m,
      config_.information_gain_ray_step_cells,
      config_.minimum_visible_unknown_cells),
  selection_policy_(
      FrontierSelectionCoreConfig{
          config_.max_retry_count,
          config_.max_cluster_retry_count,
          config_.defer_small_clusters,
          config_.small_cluster_size_threshold,
          config_.require_reachable_goal},
      config_.scoring_weights,
      std::move(ranker))
{
}

void FrontierStrategyPolicy::reset()
{
    selection_policy_.reset();
    cleanup_mode_ = false;
    no_candidate_cycles_ = 0;
}

bool FrontierStrategyPolicy::should_enter_cleanup(
    const std::vector<FrontierCluster> & clusters) const
{
    if (!config_.cleanup_enabled || clusters.empty()) {
        return false;
    }
    if (cleanup_mode_) {
        return true;
    }
    if (config_.cleanup_trigger_only_small_clusters &&
        std::all_of(
            clusters.begin(),
            clusters.end(),
            [this](const FrontierCluster & cluster) {
                return cluster.cells.size() < config_.min_cluster_size;
            }))
    {
        return true;
    }
    return no_candidate_cycles_ >= config_.cleanup_trigger_no_candidate_cycles;
}

FrontierStrategyEvaluation FrontierStrategyPolicy::evaluate(
    const grid_map_core::GridMap & map,
    const GridCell & robot_cell,
    const FrontierPruningEnvironment & environment,
    const FrontierSelectionPolicy::ReachabilityCheck & reachability_check)
{
    FrontierStrategyEvaluation evaluation;
    const auto detection_started = std::chrono::steady_clock::now();
    const auto frontier_cells = detector_.detect_frontier_cells(map);
    evaluation.clusters = detector_.cluster_frontiers(map, frontier_cells);
    evaluation.diagnostics.raw_frontier_cells = frontier_cells.size();
    evaluation.diagnostics.raw_clusters = evaluation.clusters.size();
    evaluation.detection_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - detection_started).count();
    if (evaluation.clusters.empty()) {
        evaluation.decision = {
            exploration_core::ExplorationDecisionType::WAIT,
            std::nullopt,
            "no_frontier_found"};
        return evaluation;
    }

    if (cleanup_mode_ && std::any_of(
            evaluation.clusters.begin(),
            evaluation.clusters.end(),
            [this](const FrontierCluster & cluster) {
                return cluster.cells.size() >= config_.min_cluster_size;
            }))
    {
        // 地图重新出现正常规模 frontier，说明探索重新获得了有效进展，
        // 后续恢复正常门禁，避免 cleanup 的放宽规则长期影响主流程。
        cleanup_mode_ = false;
    }

    const auto pruning_started = std::chrono::steady_clock::now();
    std::vector<GridCell> normal_failed_cluster_ids;
    auto candidates = pruner_.prune_clusters(
        evaluation.clusters,
        robot_cell,
        map.resolution,
        environment,
        selection_policy_.pruning_context(),
        &normal_failed_cluster_ids,
        &evaluation.diagnostics);
    if (candidates.empty()) {
        ++no_candidate_cycles_;
    } else {
        no_candidate_cycles_ = 0;
    }

    if (candidates.empty() && should_enter_cleanup(evaluation.clusters)) {
        cleanup_mode_ = true;
        FrontierPruner cleanup_pruner(
            config_.min_goal_distance_m,
            config_.max_retry_count,
            config_.max_cluster_retry_count,
            config_.cleanup_min_cluster_size,
            config_.unknown_margin_cells,
            config_.goal_inset_cells,
            config_.cleanup_max_unknown_ratio,
            config_.viewpoint_retreat_distances_m,
            config_.viewpoint_sample_radii_m,
            config_.viewpoint_angle_step_deg,
            config_.sensor_range_m,
            config_.information_gain_ray_step_cells,
            config_.minimum_visible_unknown_cells);
        std::vector<GridCell> cleanup_failed_cluster_ids;
        candidates = cleanup_pruner.prune_clusters(
            evaluation.clusters,
            robot_cell,
            map.resolution,
            environment,
            selection_policy_.pruning_context(),
            &cleanup_failed_cluster_ids,
            &evaluation.diagnostics);
        evaluation.failed_cluster_ids = std::move(cleanup_failed_cluster_ids);
    } else {
        evaluation.failed_cluster_ids = std::move(normal_failed_cluster_ids);
    }
    evaluation.cleanup_mode = cleanup_mode_;
    evaluation.pruning_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - pruning_started).count();
    // 候选生成失败属于本轮门禁诊断，不等同于导航失败，不在这里累计 cluster 黑名单。

    const auto ranking_started = std::chrono::steady_clock::now();
    const auto selected = selection_policy_.choose_best_candidate(candidates, reachability_check);
    evaluation.ranking_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - ranking_started).count();
    evaluation.scored_candidates = selection_policy_.last_scored_candidates();
    if (!selected.has_value()) {
        evaluation.decision = {
            exploration_core::ExplorationDecisionType::STUCK,
            std::nullopt,
            "no_valid_frontier"};
        return evaluation;
    }

    selection_policy_.set_last_goal(selected->candidate.goal);
    evaluation.decision = {
        exploration_core::ExplorationDecisionType::NAVIGATE,
        selected->candidate.goal,
        "frontier_goal_selected"};
    return evaluation;
}

void FrontierStrategyPolicy::on_outcome(
    const exploration_core::ExplorationOutcome & outcome)
{
    if (!outcome.goal.has_value()) {
        return;
    }

    switch (outcome.type) {
        case exploration_core::ExplorationOutcomeType::NAVIGATION_SUCCEEDED:
            selection_policy_.mark_goal_succeeded(outcome.goal.value());
            break;
        case exploration_core::ExplorationOutcomeType::NAVIGATION_FAILED:
            selection_policy_.mark_goal_failed(outcome.goal.value());
            break;
        case exploration_core::ExplorationOutcomeType::CANCELED:
            break;
    }
}

void FrontierStrategyPolicy::mark_goal_failed(const GridCell & goal)
{
    selection_policy_.mark_goal_failed(goal);
}

void FrontierStrategyPolicy::mark_goal_succeeded(const GridCell & goal)
{
    selection_policy_.mark_goal_succeeded(goal);
}

std::vector<GridCell> FrontierStrategyPolicy::blacklisted_goals() const
{
    return selection_policy_.blacklisted_goals();
}

int FrontierStrategyPolicy::retry_count_for_goal(const GridCell & goal) const
{
    return selection_policy_.retry_count_for_goal(goal);
}

bool FrontierStrategyPolicy::is_goal_blacklisted(const GridCell & goal) const
{
    return selection_policy_.is_goal_blacklisted(goal);
}

std::size_t FrontierStrategyPolicy::clear_blacklist()
{
    return selection_policy_.clear_blacklist();
}

}  // 命名空间 frontier_strategy

