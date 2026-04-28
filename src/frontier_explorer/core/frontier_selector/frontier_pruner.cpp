#include "frontier_pruner.hpp"

#include <algorithm>
#include <limits>

#include "frontier_selector_utils.hpp"

namespace frontier_explorer
{

FrontierPruner::FrontierPruner(
    double min_goal_distance_m,
    int max_retry_count,
    int max_cluster_retry_count,
    std::size_t min_cluster_size,
    int unknown_margin_cells,
    const rclcpp::Logger & logger)
: logger_(rclcpp::Logger(logger).get_child("pruner")),
  min_goal_distance_m_(min_goal_distance_m),
  max_retry_count_(max_retry_count),
  max_cluster_retry_count_(max_cluster_retry_count),
  min_cluster_size_(min_cluster_size),
  unknown_margin_cells_(std::max(0, unknown_margin_cells))
{
}

bool FrontierPruner::is_same_as_last_goal(
    const GridCell & goal,
    const std::optional<GridCell> & last_goal) const
{
    return last_goal.has_value() && last_goal.value() == goal;
}

int FrontierPruner::retry_count_of_goal(
    const GridCell & goal,
    const FrontierPruningContext & context) const
{
    if (context.failed_goal_counts == nullptr) {
        return 0;
    }

    const auto it = context.failed_goal_counts->find(goal);
    if (it == context.failed_goal_counts->end()) {
        return 0;
    }

    return it->second;
}

bool FrontierPruner::should_skip_goal(
    const GridCell & goal,
    const FrontierPruningContext & context) const
{
    if (context.goal_blacklist != nullptr && context.goal_blacklist->count(goal) > 0) {
        return true;
    }

    const int retry_count = retry_count_of_goal(goal, context);
    return retry_count >= max_retry_count_;
}

std::optional<GridCell> FrontierPruner::find_fallback_goal_in_cluster(
    const FrontierCluster & cluster,
    const GridCell & robot_grid,
    double resolution,
    const CostmapAdapter * frontier_costmap,
    const FrontierPruningContext & context) const
{
    double best_dist = std::numeric_limits<double>::max();
    std::optional<GridCell> best_cell;

    for (const auto & cell : cluster.cells) {
        if (should_skip_goal(cell, context)) {
            continue;
        }
        if (is_same_as_last_goal(cell, context.last_goal)) {
            continue;
        }
        if (!pass_map_candidate_constraints(cell, frontier_costmap)) {
            continue;
        }

        const double dist_m = grid_distance_in_meters(robot_grid, cell, resolution);
        if (dist_m < min_goal_distance_m_) {
            continue;
        }

        if (dist_m < best_dist) {
            best_dist = dist_m;
            best_cell = cell;
        }
    }

    return best_cell;
}

bool FrontierPruner::pass_map_candidate_constraints(
    const GridCell & cell,
    const CostmapAdapter * frontier_costmap,
    double * unknown_ratio) const
{
    if (unknown_ratio != nullptr) {
        *unknown_ratio = 0.0;
    }

    if (frontier_costmap == nullptr) {
        return true;
    }

    if (!frontier_costmap->inBounds(cell.col, cell.row) ||
        !frontier_costmap->isFree(
            static_cast<unsigned int>(cell.col),
            static_cast<unsigned int>(cell.row)))
    {
        return false;
    }

    if (unknown_margin_cells_ <= 0) {
        return true;
    }

    int unknown_count = 0;
    int observed_count = 0;
    for (int dr = -unknown_margin_cells_; dr <= unknown_margin_cells_; ++dr) {
        for (int dc = -unknown_margin_cells_; dc <= unknown_margin_cells_; ++dc) {
            const int row = cell.row + dr;
            const int col = cell.col + dc;
            if (!frontier_costmap->inBounds(col, row)) {
                continue;
            }

            ++observed_count;
            if (frontier_costmap->isUnknown(
                static_cast<unsigned int>(col),
                static_cast<unsigned int>(row)))
            {
                ++unknown_count;
            }
        }
    }

    if (observed_count <= 0) {
        return false;
    }

    const double ratio =
        static_cast<double>(unknown_count) / static_cast<double>(observed_count);
    if (unknown_ratio != nullptr) {
        *unknown_ratio = ratio;
    }
    return true;
}

double FrontierPruner::compute_clearance_m(
    const GridCell & cell,
    const CostmapAdapter * frontier_costmap,
    const CostmapAdapter * safety_costmap) const
{
    const CostmapAdapter * query_costmap =
        safety_costmap != nullptr ? safety_costmap : frontier_costmap;
    if (query_costmap == nullptr || frontier_costmap == nullptr) {
        return 0.0;
    }

    if (!frontier_costmap->inBounds(cell.col, cell.row)) {
        RCLCPP_DEBUG(logger_, "Clearance query skipped for out-of-bounds candidate.");
        return 0.0;
    }

    unsigned int query_col = static_cast<unsigned int>(cell.col);
    unsigned int query_row = static_cast<unsigned int>(cell.row);
    if (query_costmap != frontier_costmap) {
        double wx = 0.0;
        double wy = 0.0;
        frontier_costmap->mapToWorld(
            static_cast<unsigned int>(cell.col),
            static_cast<unsigned int>(cell.row),
            wx,
            wy);
        if (!query_costmap->worldToMap(wx, wy, query_col, query_row)) {
            RCLCPP_DEBUG(logger_, "Clearance query failed because safety map conversion failed.");
            return 0.0;
        }
    }

    const auto clearance = query_costmap->distanceToNearestObstacle(
        query_col,
        query_row,
        unknown_margin_cells_);
    if (!clearance.has_value()) {
        RCLCPP_DEBUG(logger_, "No obstacle found in local clearance search window.");
        return static_cast<double>(unknown_margin_cells_) * query_costmap->getResolution();
    }
    return clearance.value();
}

std::vector<FrontierCandidate> FrontierPruner::prune_clusters(
    const std::vector<FrontierCluster> & clusters,
    const GridCell & robot_grid,
    double resolution,
    const CostmapAdapter * frontier_costmap,
    const CostmapAdapter * safety_costmap,
    const FrontierPruningContext & context,
    std::vector<GridCell> * failed_cluster_ids) const
{
    std::vector<FrontierCandidate> valid_candidates;
    valid_candidates.reserve(clusters.size());

    for (std::size_t cluster_idx = 0; cluster_idx < clusters.size(); ++cluster_idx) {
        const auto & cluster = clusters[cluster_idx];

        if (cluster.cells.size() < min_cluster_size_) {
            continue;
        }

        if (context.cluster_blacklist != nullptr &&
            context.cluster_blacklist->count(cluster.centroid) > 0)
        {
            continue;
        }

        if (context.failed_cluster_counts != nullptr) {
            const auto cluster_retry_it = context.failed_cluster_counts->find(cluster.centroid);
            if (cluster_retry_it != context.failed_cluster_counts->end() &&
                cluster_retry_it->second >= max_cluster_retry_count_)
            {
                continue;
            }
        }

        GridCell candidate = cluster.centroid;
        double dist_m = grid_distance_in_meters(robot_grid, candidate, resolution);
        double unknown_ratio = 0.0;
        bool used_fallback = false;

        bool centroid_invalid = false;
        if (should_skip_goal(candidate, context)) {
            centroid_invalid = true;
        } else if (dist_m < min_goal_distance_m_) {
            centroid_invalid = true;
        } else if (is_same_as_last_goal(candidate, context.last_goal)) {
            centroid_invalid = true;
        } else if (!pass_map_candidate_constraints(candidate, frontier_costmap, &unknown_ratio)) {
            centroid_invalid = true;
        }

        if (centroid_invalid) {
            const auto fallback = find_fallback_goal_in_cluster(
                cluster, robot_grid, resolution, frontier_costmap, context);
            if (!fallback.has_value()) {
                if (failed_cluster_ids != nullptr) {
                    failed_cluster_ids->push_back(cluster.centroid);
                }
                continue;
            }

            candidate = fallback.value();
            dist_m = grid_distance_in_meters(robot_grid, candidate, resolution);
            pass_map_candidate_constraints(candidate, frontier_costmap, &unknown_ratio);
            used_fallback = true;
        }

        const double clearance_m = compute_clearance_m(
            candidate,
            frontier_costmap,
            safety_costmap);

        valid_candidates.push_back(FrontierCandidate{
            candidate,
            cluster.centroid,
            cluster.cells.size(),
            dist_m,
            retry_count_of_goal(candidate, context),
            clearance_m,
            unknown_ratio,
            cluster_idx,
            used_fallback
        });
    }

    return valid_candidates;
}

}  // namespace frontier_explorer
