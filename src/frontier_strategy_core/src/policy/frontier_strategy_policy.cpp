#include "frontier_strategy_core/policy/frontier_strategy_policy.hpp"

#include <utility>

namespace frontier_strategy
{

FrontierStrategyPolicy::FrontierStrategyPolicy(
    FrontierStrategyPolicyConfig config)
: config_(std::move(config)),
  detector_(config_.obstacle_search_radius_cells),
  pruner_(
      config_.min_goal_distance_m,
      config_.max_retry_count,
      config_.max_cluster_retry_count,
      config_.min_cluster_size,
      config_.unknown_margin_cells,
      config_.goal_inset_cells,
      config_.max_unknown_ratio),
  selection_policy_(
      FrontierSelectionCoreConfig{
          config_.max_retry_count,
          config_.max_cluster_retry_count,
          config_.defer_small_clusters,
          config_.small_cluster_size_threshold,
          config_.require_reachable_goal},
      config_.scoring_weights)
{
}

void FrontierStrategyPolicy::reset()
{
    selection_policy_.reset();
}

exploration_core::ExplorationDecision FrontierStrategyPolicy::decide(
    const exploration_core::ExplorationObservation & observation)
{
    if (!observation.mapping_active) {
        return {
            exploration_core::ExplorationDecisionType::WAIT,
            std::nullopt,
            "mapping_not_active"};
    }
    if (!observation.map.has_value() || !observation.map->isReady()) {
        return {
            exploration_core::ExplorationDecisionType::WAIT,
            std::nullopt,
            "map_not_ready"};
    }
    if (!observation.robot_cell.has_value()) {
        return {
            exploration_core::ExplorationDecisionType::WAIT,
            std::nullopt,
            "robot_cell_unavailable"};
    }

    FrontierPruningEnvironment environment;
    environment.frontier_map = &observation.map.value();
    return evaluate(
        observation.map.value(),
        observation.robot_cell.value(),
        environment).decision;
}

FrontierStrategyEvaluation FrontierStrategyPolicy::evaluate(
    const grid_map_core::GridMap & map,
    const GridCell & robot_cell,
    const FrontierPruningEnvironment & environment,
    const FrontierSelectionPolicy::ReachabilityCheck & reachability_check)
{
    FrontierStrategyEvaluation evaluation;
    const auto frontier_cells = detector_.detect_frontier_cells(map);
    evaluation.clusters = detector_.cluster_frontiers(map, frontier_cells);
    if (evaluation.clusters.empty()) {
        evaluation.decision = {
            exploration_core::ExplorationDecisionType::COMPLETED,
            std::nullopt,
            "no_frontier_found"};
        return evaluation;
    }

    const auto candidates = pruner_.prune_clusters(
        evaluation.clusters,
        robot_cell,
        map.resolution,
        environment,
        selection_policy_.pruning_context(),
        &evaluation.failed_cluster_ids);
    selection_policy_.record_failed_clusters(evaluation.failed_cluster_ids);
    const auto selected = selection_policy_.choose_best_candidate(candidates, reachability_check);
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

