#include "core/selector/filters/frontier_pruner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_set>

#include "core/utils/frontier_selector_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_explorer
{
namespace
{
constexpr std::array<double, 2U> kRetreatDistancesM{0.25, 0.4};
constexpr std::array<double, 2U> kSampleRadiiM{0.35, 0.55};
constexpr double kAngleStepDeg = 30.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
}  // namespace

FrontierPruner::FrontierPruner(
    double min_goal_distance_m,
    int max_retry_count,
    int max_cluster_retry_count,
    std::size_t min_cluster_size,
    int unknown_margin_cells,
    int goal_inset_cells,
    double max_unknown_ratio,
    const FootprintCollisionCheckerConfig & footprint_collision_config,
    const rclcpp::Logger & logger)
: logger_(rclcpp::Logger(logger).get_child("pruner")),
  min_goal_distance_m_(min_goal_distance_m),
  max_retry_count_(max_retry_count),
  max_cluster_retry_count_(max_cluster_retry_count),
  min_cluster_size_(min_cluster_size),
  unknown_margin_cells_(std::max(0, unknown_margin_cells)),
  goal_inset_cells_(std::max(0, goal_inset_cells)),
  max_unknown_ratio_(std::clamp(max_unknown_ratio, 0.0, 1.0)),
  footprint_collision_config_(footprint_collision_config)
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
    return ratio <= max_unknown_ratio_;
}

bool FrontierPruner::pass_safety_candidate_constraints(
    const GridCell & cell,
    const CostmapAdapter * frontier_costmap,
    const CostmapAdapter * safety_costmap) const
{
    if (frontier_costmap == nullptr || safety_costmap == nullptr || !safety_costmap->isReady()) {
        return true;
    }

    double wx = 0.0;
    double wy = 0.0;
    frontier_costmap->mapToWorld(
        static_cast<unsigned int>(cell.col),
        static_cast<unsigned int>(cell.row),
        wx,
        wy);

    unsigned int safety_col = 0U;
    unsigned int safety_row = 0U;
    if (!safety_costmap->worldToMap(wx, wy, safety_col, safety_row)) {
        return false;
    }

    const auto cost = safety_costmap->getCost(safety_col, safety_row);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
        return false;
    }
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
    }

    const auto footprint_result = FootprintCollisionChecker::checkWorldPoint(
        *safety_costmap,
        wx,
        wy,
        0.0,
        footprint_collision_config_);
    if (!footprint_result.valid) {
        RCLCPP_DEBUG(
            logger_,
            "Candidate rejected by footprint hard filter: goal=(%d, %d), reason=%s, max_cost=%.1f",
            cell.row,
            cell.col,
            footprint_result.reason.c_str(),
            footprint_result.max_cost);
        return false;
    }
    return true;
}

GridCell FrontierPruner::inset_goal_toward_robot(
    const GridCell & frontier_goal,
    const GridCell & robot_grid,
    const CostmapAdapter * frontier_costmap,
    const FrontierPruningContext & context) const
{
    if (goal_inset_cells_ <= 0 || frontier_costmap == nullptr) {
        return frontier_goal;
    }

    const int delta_row = robot_grid.row - frontier_goal.row;
    const int delta_col = robot_grid.col - frontier_goal.col;
    if (delta_row == 0 && delta_col == 0) {
        return frontier_goal;
    }

    const int step_row = delta_row == 0 ? 0 : (delta_row > 0 ? 1 : -1);
    const int step_col = delta_col == 0 ? 0 : (delta_col > 0 ? 1 : -1);
    GridCell best = frontier_goal;
    for (int step = 1; step <= goal_inset_cells_; ++step) {
        const GridCell inset{
            frontier_goal.row + step_row * step,
            frontier_goal.col + step_col * step};
        if (should_skip_goal(inset, context)) {
            continue;
        }
        if (!pass_map_candidate_constraints(inset, frontier_costmap)) {
            continue;
        }
        best = inset;
    }

    return best;
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
    valid_candidates.reserve(clusters.size() * 4U);
    std::unordered_set<GridCell, GridCellHash> emitted_goals;

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

        bool cluster_generated_candidate = false;
        auto append_candidate =
            [&](GridCell candidate, bool used_fallback, bool allow_inset) {
                if (should_skip_goal(candidate, context)) {
                    return false;
                }
                if (is_same_as_last_goal(candidate, context.last_goal)) {
                    return false;
                }

                double dist_m = grid_distance_in_meters(robot_grid, candidate, resolution);
                if (dist_m < min_goal_distance_m_) {
                    return false;
                }

                double unknown_ratio = 0.0;
                if (!pass_map_candidate_constraints(candidate, frontier_costmap, &unknown_ratio)) {
                    return false;
                }
                if (!pass_safety_candidate_constraints(
                        candidate,
                        frontier_costmap,
                        safety_costmap))
                {
                    return false;
                }

                bool goal_inset_applied = false;
                if (allow_inset) {
                    const GridCell original_candidate = candidate;
                    candidate = inset_goal_toward_robot(
                        candidate,
                        robot_grid,
                        frontier_costmap,
                        context);
                    goal_inset_applied = !(candidate == original_candidate);
                    if (goal_inset_applied) {
                        if (should_skip_goal(candidate, context) ||
                            is_same_as_last_goal(candidate, context.last_goal))
                        {
                            return false;
                        }
                        dist_m = grid_distance_in_meters(robot_grid, candidate, resolution);
                        if (dist_m < min_goal_distance_m_) {
                            return false;
                        }
                        if (!pass_map_candidate_constraints(
                                candidate,
                                frontier_costmap,
                                &unknown_ratio))
                        {
                            return false;
                        }
                        if (!pass_safety_candidate_constraints(
                                candidate,
                                frontier_costmap,
                                safety_costmap))
                        {
                            return false;
                        }
                    }
                }

                if (emitted_goals.count(candidate) > 0U) {
                    return false;
                }
                emitted_goals.insert(candidate);

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
                    used_fallback,
                    goal_inset_applied,
                    false,
                    true,
                    0.0
                });
                return true;
            };

        cluster_generated_candidate |= append_candidate(cluster.centroid, false, true);

        const auto fallback = find_fallback_goal_in_cluster(
            cluster,
            robot_grid,
            resolution,
            frontier_costmap,
            context);
        if (fallback.has_value()) {
            cluster_generated_candidate |= append_candidate(fallback.value(), true, true);
        }

        if (frontier_costmap != nullptr && frontier_costmap->isReady()) {
            double centroid_x = 0.0;
            double centroid_y = 0.0;
            double robot_x = 0.0;
            double robot_y = 0.0;
            frontier_costmap->mapToWorld(
                static_cast<unsigned int>(cluster.centroid.col),
                static_cast<unsigned int>(cluster.centroid.row),
                centroid_x,
                centroid_y);
            frontier_costmap->mapToWorld(
                static_cast<unsigned int>(robot_grid.col),
                static_cast<unsigned int>(robot_grid.row),
                robot_x,
                robot_y);

            const double direction_x = robot_x - centroid_x;
            const double direction_y = robot_y - centroid_y;
            const double direction_norm = std::hypot(direction_x, direction_y);
            if (direction_norm > 1e-6) {
                for (const auto retreat_distance_m : kRetreatDistancesM) {
                    unsigned int mx = 0U;
                    unsigned int my = 0U;
                    const double wx =
                        centroid_x + direction_x / direction_norm * retreat_distance_m;
                    const double wy =
                        centroid_y + direction_y / direction_norm * retreat_distance_m;
                    if (frontier_costmap->worldToMap(wx, wy, mx, my)) {
                        cluster_generated_candidate |= append_candidate(
                            GridCell{static_cast<int>(my), static_cast<int>(mx)},
                            true,
                            false);
                    }
                }
            }

            for (const auto sample_radius_m : kSampleRadiiM) {
                for (double angle_deg = 0.0; angle_deg < 360.0; angle_deg += kAngleStepDeg) {
                    const double angle_rad = angle_deg * kDegreesToRadians;
                    unsigned int mx = 0U;
                    unsigned int my = 0U;
                    const double wx = centroid_x + std::cos(angle_rad) * sample_radius_m;
                    const double wy = centroid_y + std::sin(angle_rad) * sample_radius_m;
                    if (frontier_costmap->worldToMap(wx, wy, mx, my)) {
                        cluster_generated_candidate |= append_candidate(
                            GridCell{static_cast<int>(my), static_cast<int>(mx)},
                            true,
                            false);
                    }
                }
            }
        }

        if (!cluster_generated_candidate && failed_cluster_ids != nullptr) {
            failed_cluster_ids->push_back(cluster.centroid);
        }
    }

    return valid_candidates;
}

}  // namespace frontier_explorer
