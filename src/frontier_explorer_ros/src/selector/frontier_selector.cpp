#include "frontier_explorer_ros/selector/frontier_selector.hpp"

#include <algorithm>
#include <optional>
#include <sstream>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_explorer
{

FrontierSelector::FrontierSelector(
    double min_goal_distance_m,
    int max_retry_count,
    const FrontierScoringWeights & scoring_weights,
    std::size_t min_cluster_size,
    int max_cluster_retry_count,
    int candidate_unknown_margin_cells,
    int candidate_goal_inset_cells,
    double candidate_max_unknown_ratio,
    const FootprintCollisionCheckerConfig & footprint_collision_config,
    bool defer_small_clusters,
    std::size_t small_cluster_size_threshold,
    bool require_reachable_goal,
    const rclcpp::Logger & logger)
: logger_(rclcpp::Logger(logger).get_child("selector")),
  max_retry_count_(max_retry_count),
  candidate_unknown_margin_cells_(std::max(0, candidate_unknown_margin_cells)),
  footprint_collision_config_(footprint_collision_config),
  pruner_(
      min_goal_distance_m,
      max_retry_count,
      max_cluster_retry_count,
      std::max<std::size_t>(1U, min_cluster_size),
      candidate_unknown_margin_cells_,
      candidate_goal_inset_cells,
      candidate_max_unknown_ratio),
  selection_policy_(
      FrontierSelectionCoreConfig{
          max_retry_count,
          max_cluster_retry_count,
          defer_small_clusters,
          std::max<std::size_t>(
              std::max<std::size_t>(1U, min_cluster_size),
              small_cluster_size_threshold),
          require_reachable_goal},
      scoring_weights)
{
}

FrontierPruningEnvironment FrontierSelector::make_pruning_environment(
    const CostmapAdapter & frontier_costmap,
    const CostmapAdapter * safety_costmap) const
{
    FrontierPruningEnvironment environment;
    environment.frontier_map = &frontier_costmap.gridMap();
    environment.safety_check =
        [this, &frontier_costmap, safety_costmap](const GridCell & cell) {
            if (safety_costmap == nullptr || !safety_costmap->isReady()) {
                return true;
            }

            double world_x = 0.0;
            double world_y = 0.0;
            frontier_costmap.mapToWorld(
                static_cast<unsigned int>(cell.col),
                static_cast<unsigned int>(cell.row),
                world_x,
                world_y);

            unsigned int safety_col = 0U;
            unsigned int safety_row = 0U;
            if (!safety_costmap->worldToMap(
                    world_x,
                    world_y,
                    safety_col,
                    safety_row))
            {
                return false;
            }

            const auto cost = safety_costmap->getCost(safety_col, safety_row);
            if (cost == nav2_costmap_2d::NO_INFORMATION ||
                cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                return false;
            }

            const auto footprint_result = FootprintCollisionChecker::checkWorldPoint(
                *safety_costmap,
                world_x,
                world_y,
                0.0,
                footprint_collision_config_);
            if (!footprint_result.valid) {
                RCLCPP_DEBUG(
                    logger_,
                    "Candidate rejected by footprint hard filter: "
                    "goal=(%d, %d), reason=%s, max_cost=%.1f",
                    cell.row,
                    cell.col,
                    footprint_result.reason.c_str(),
                    footprint_result.max_cost);
                return false;
            }
            return true;
        };

    environment.clearance_query =
        [this, &frontier_costmap, safety_costmap](
            const GridCell & cell) -> std::optional<double> {
            const CostmapAdapter * query_costmap =
                safety_costmap != nullptr ? safety_costmap : &frontier_costmap;
            if (!frontier_costmap.inBounds(cell.col, cell.row)) {
                RCLCPP_DEBUG(logger_, "Clearance query skipped for out-of-bounds candidate.");
                return 0.0;
            }

            unsigned int query_col = static_cast<unsigned int>(cell.col);
            unsigned int query_row = static_cast<unsigned int>(cell.row);
            if (query_costmap != &frontier_costmap) {
                double world_x = 0.0;
                double world_y = 0.0;
                frontier_costmap.mapToWorld(
                    static_cast<unsigned int>(cell.col),
                    static_cast<unsigned int>(cell.row),
                    world_x,
                    world_y);
                if (!query_costmap->worldToMap(
                        world_x,
                        world_y,
                        query_col,
                        query_row))
                {
                    RCLCPP_DEBUG(
                        logger_,
                        "Clearance query failed because safety map conversion failed.");
                    return 0.0;
                }
            }

            const auto clearance = query_costmap->distanceToNearestObstacle(
                query_col,
                query_row,
                candidate_unknown_margin_cells_);
            if (!clearance.has_value()) {
                RCLCPP_DEBUG(logger_, "No obstacle found in local clearance search window.");
                return static_cast<double>(candidate_unknown_margin_cells_) *
                       query_costmap->getResolution();
            }
            return clearance.value();
        };
    return environment;
}

std::optional<GridCell> FrontierSelector::choose_best_frontier(
    const std::vector<FrontierCluster> & clusters,
    const GridCell & robot_grid,
    double resolution,
    const CostmapAdapter & frontier_costmap,
    const CostmapAdapter * safety_costmap,
    const std::function<FrontierReachabilityResult(FrontierCandidate &)> &
        reachability_check)
{
    std::vector<GridCell> failed_clusters;
    const auto candidates = pruner_.prune_clusters(
        clusters,
        robot_grid,
        resolution,
        make_pruning_environment(frontier_costmap, safety_costmap),
        selection_policy_.pruning_context(),
        &failed_clusters);
    selection_policy_.record_failed_clusters(failed_clusters);

    if (candidates.empty()) {
        RCLCPP_WARN(logger_, "No valid frontier candidates after pruning.");
        return std::nullopt;
    }

    const auto selected =
        selection_policy_.choose_best_candidate(candidates, reachability_check);
    log_reachability_failures(selection_policy_.last_scored_candidates());
    if (!selected.has_value()) {
        RCLCPP_WARN(logger_, "No reachable frontier candidates after planner feasibility check.");
        return std::nullopt;
    }

    log_scored_candidates(selection_policy_.last_scored_candidates(), selected.value());
    return selected->candidate.goal;
}

std::vector<ScoredFrontierCandidate> FrontierSelector::rank_frontier_candidates(
    const std::vector<FrontierCluster> & clusters,
    const GridCell & robot_grid,
    double resolution,
    const CostmapAdapter & frontier_costmap,
    const CostmapAdapter * safety_costmap,
    const std::function<FrontierReachabilityResult(FrontierCandidate &)> &
        reachability_check)
{
    std::vector<GridCell> failed_clusters;
    const auto candidates = pruner_.prune_clusters(
        clusters,
        robot_grid,
        resolution,
        make_pruning_environment(frontier_costmap, safety_costmap),
        selection_policy_.pruning_context(),
        &failed_clusters);
    selection_policy_.record_failed_clusters(failed_clusters);

    if (candidates.empty()) {
        RCLCPP_WARN(logger_, "No valid frontier candidates after pruning.");
        return {};
    }

    auto scored = selection_policy_.rank_candidates(candidates, reachability_check);
    log_reachability_failures(scored);
    return scored;
}

void FrontierSelector::set_last_goal(const GridCell & goal)
{
    selection_policy_.set_last_goal(goal);
}

void FrontierSelector::mark_goal_failed(const GridCell & goal)
{
    selection_policy_.mark_goal_failed(goal);
    const int count = selection_policy_.retry_count_for_goal(goal);
    if (selection_policy_.is_goal_blacklisted(goal)) {
        RCLCPP_WARN(
            logger_,
            "Goal blacklisted: row=%d, col=%d, failures=%d, threshold=%d",
            goal.row,
            goal.col,
            count,
            max_retry_count_);
    } else {
        RCLCPP_WARN(
            logger_,
            "Goal failed: row=%d, col=%d, failures=%d/%d",
            goal.row,
            goal.col,
            count,
            max_retry_count_);
    }
}

void FrontierSelector::mark_goal_succeeded(const GridCell & goal)
{
    selection_policy_.mark_goal_succeeded(goal);
}

const std::vector<ScoredFrontierCandidate> & FrontierSelector::last_scored_candidates() const
{
    return selection_policy_.last_scored_candidates();
}

std::vector<GridCell> FrontierSelector::blacklisted_goals() const
{
    return selection_policy_.blacklisted_goals();
}

int FrontierSelector::retry_count_for_goal(const GridCell & goal) const
{
    return selection_policy_.retry_count_for_goal(goal);
}

bool FrontierSelector::is_goal_blacklisted(const GridCell & goal) const
{
    return selection_policy_.is_goal_blacklisted(goal);
}

std::size_t FrontierSelector::clear_blacklist()
{
    return selection_policy_.clear_blacklist();
}

void FrontierSelector::log_reachability_failures(
    const std::vector<ScoredFrontierCandidate> & scored_candidates) const
{
    for (const auto & scored : scored_candidates) {
        const auto & candidate = scored.candidate;
        if (candidate.reachability_reason.empty()) {
            continue;
        }
        RCLCPP_DEBUG(
            logger_,
            "Frontier reachability diagnostic: goal=(%d, %d), checked=%s, "
            "reachable=%s, reason=%s",
            candidate.goal.row,
            candidate.goal.col,
            candidate.reachability_checked ? "true" : "false",
            candidate.reachable ? "true" : "false",
            candidate.reachability_reason.c_str());
    }
}

void FrontierSelector::log_scored_candidates(
    const std::vector<ScoredFrontierCandidate> & scored_candidates,
    const ScoredFrontierCandidate & selected) const
{
    std::vector<ScoredFrontierCandidate> sorted = scored_candidates;
    std::sort(
        sorted.begin(),
        sorted.end(),
        [](const ScoredFrontierCandidate & lhs, const ScoredFrontierCandidate & rhs) {
            return lhs.total_score > rhs.total_score;
        });

    std::ostringstream stream;
    stream.precision(3);
    stream << std::fixed
           << "Selected frontier reason: goal=("
           << selected.candidate.goal.row << ", " << selected.candidate.goal.col
           << "), total=" << selected.total_score
           << ", distance_score=" << selected.distance_score
           << ", cluster_score=" << selected.cluster_size_score
           << ", clearance_score=" << selected.clearance_score
           << ", retry_penalty=" << selected.retry_penalty
           << ", unknown_risk_penalty=" << selected.unknown_risk_penalty
           << ", information_gain=" << selected.information_gain_score
           << ", distance_m=" << selected.candidate.distance_m
           << ", cluster_size=" << selected.candidate.cluster_size
           << ", clearance_m=" << selected.candidate.clearance_m
           << ", unknown_ratio=" << selected.candidate.unknown_ratio
           << ", retry_count=" << selected.candidate.retry_count
           << ", used_fallback=" << (selected.candidate.used_fallback ? "true" : "false");

    const std::size_t top_count = std::min<std::size_t>(3U, sorted.size());
    for (std::size_t index = 0; index < top_count; ++index) {
        const auto & scored = sorted[index];
        stream << " | top" << (index + 1U)
               << " goal=(" << scored.candidate.goal.row << ", "
               << scored.candidate.goal.col
               << ") total=" << scored.total_score
               << " d=" << scored.distance_score
               << " c=" << scored.cluster_size_score
               << " clear=" << scored.clearance_score
               << " risk=-" << scored.unknown_risk_penalty
               << " retry=-" << scored.retry_penalty;
    }

    RCLCPP_INFO(logger_, "%s", stream.str().c_str());
}

}  // namespace frontier_explorer
