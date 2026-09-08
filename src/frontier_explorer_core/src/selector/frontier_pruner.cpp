#include "frontier_explorer_core/selector/filters/frontier_pruner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_set>

#include "frontier_explorer_core/utils/frontier_selector_utils.hpp"

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
    double max_unknown_ratio)
: min_goal_distance_m_(min_goal_distance_m),
  max_retry_count_(max_retry_count),
  max_cluster_retry_count_(max_cluster_retry_count),
  min_cluster_size_(min_cluster_size),
  unknown_margin_cells_(std::max(0, unknown_margin_cells)),
  goal_inset_cells_(std::max(0, goal_inset_cells)),
  max_unknown_ratio_(std::clamp(max_unknown_ratio, 0.0, 1.0))
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
    return it == context.failed_goal_counts->end() ? 0 : it->second;
}

bool FrontierPruner::should_skip_goal(
    const GridCell & goal,
    const FrontierPruningContext & context) const
{
    if (context.goal_blacklist != nullptr && context.goal_blacklist->count(goal) > 0U) {
        return true;
    }
    return retry_count_of_goal(goal, context) >= max_retry_count_;
}

std::optional<GridCell> FrontierPruner::find_fallback_goal_in_cluster(
    const FrontierCluster & cluster,
    const GridCell & robot_grid,
    double resolution,
    const GridMap * frontier_map,
    const FrontierPruningContext & context) const
{
    double best_distance = std::numeric_limits<double>::max();
    std::optional<GridCell> best_cell;

    for (const auto & cell : cluster.cells) {
        if (should_skip_goal(cell, context) ||
            is_same_as_last_goal(cell, context.last_goal) ||
            !pass_map_candidate_constraints(cell, frontier_map))
        {
            continue;
        }

        const double distance_m = grid_distance_in_meters(robot_grid, cell, resolution);
        if (distance_m < min_goal_distance_m_) {
            continue;
        }

        if (distance_m < best_distance) {
            best_distance = distance_m;
            best_cell = cell;
        }
    }
    return best_cell;
}

bool FrontierPruner::pass_map_candidate_constraints(
    const GridCell & cell,
    const GridMap * frontier_map,
    double * unknown_ratio) const
{
    if (unknown_ratio != nullptr) {
        *unknown_ratio = 0.0;
    }
    if (frontier_map == nullptr) {
        return true;
    }
    if (!frontier_map->inBounds(cell.col, cell.row) ||
        !frontier_map->isFree(
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
    for (int row_offset = -unknown_margin_cells_;
        row_offset <= unknown_margin_cells_;
        ++row_offset)
    {
        for (int col_offset = -unknown_margin_cells_;
            col_offset <= unknown_margin_cells_;
            ++col_offset)
        {
            const int row = cell.row + row_offset;
            const int col = cell.col + col_offset;
            if (!frontier_map->inBounds(col, row)) {
                continue;
            }

            ++observed_count;
            if (frontier_map->isUnknown(
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
    const FrontierPruningEnvironment & environment) const
{
    return !environment.safety_check || environment.safety_check(cell);
}

GridCell FrontierPruner::inset_goal_toward_robot(
    const GridCell & frontier_goal,
    const GridCell & robot_grid,
    const GridMap * frontier_map,
    const FrontierPruningContext & context) const
{
    if (goal_inset_cells_ <= 0 || frontier_map == nullptr) {
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
        if (should_skip_goal(inset, context) ||
            !pass_map_candidate_constraints(inset, frontier_map))
        {
            continue;
        }
        best = inset;
    }
    return best;
}

double FrontierPruner::compute_clearance_m(
    const GridCell & cell,
    const FrontierPruningEnvironment & environment) const
{
    if (environment.clearance_query) {
        return environment.clearance_query(cell).value_or(0.0);
    }
    if (environment.frontier_map == nullptr) {
        return 0.0;
    }

    const auto clearance = environment.frontier_map->distanceToNearestObstacle(
        cell,
        unknown_margin_cells_);
    if (clearance.has_value()) {
        return clearance.value();
    }
    return static_cast<double>(unknown_margin_cells_) *
           environment.frontier_map->resolution;
}

std::vector<FrontierCandidate> FrontierPruner::prune_clusters(
    const std::vector<FrontierCluster> & clusters,
    const GridCell & robot_grid,
    double resolution,
    const FrontierPruningEnvironment & environment,
    const FrontierPruningContext & context,
    std::vector<GridCell> * failed_cluster_ids) const
{
    std::vector<FrontierCandidate> valid_candidates;
    valid_candidates.reserve(clusters.size() * 4U);
    std::unordered_set<GridCell, GridCellHash> emitted_goals;

    for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
        const auto & cluster = clusters[cluster_index];
        if (cluster.cells.size() < min_cluster_size_) {
            continue;
        }
        if (context.cluster_blacklist != nullptr &&
            context.cluster_blacklist->count(cluster.centroid) > 0U)
        {
            continue;
        }
        if (context.failed_cluster_counts != nullptr) {
            const auto retry = context.failed_cluster_counts->find(cluster.centroid);
            if (retry != context.failed_cluster_counts->end() &&
                retry->second >= max_cluster_retry_count_)
            {
                continue;
            }
        }

        bool cluster_generated_candidate = false;
        auto append_candidate =
            [&](GridCell candidate, bool used_fallback, bool allow_inset) {
                if (should_skip_goal(candidate, context) ||
                    is_same_as_last_goal(candidate, context.last_goal))
                {
                    return false;
                }

                double distance_m = grid_distance_in_meters(robot_grid, candidate, resolution);
                if (distance_m < min_goal_distance_m_) {
                    return false;
                }

                double unknown_ratio = 0.0;
                if (!pass_map_candidate_constraints(
                        candidate,
                        environment.frontier_map,
                        &unknown_ratio) ||
                    !pass_safety_candidate_constraints(candidate, environment))
                {
                    return false;
                }

                bool goal_inset_applied = false;
                if (allow_inset) {
                    const GridCell original_candidate = candidate;
                    candidate = inset_goal_toward_robot(
                        candidate,
                        robot_grid,
                        environment.frontier_map,
                        context);
                    goal_inset_applied = !(candidate == original_candidate);
                    if (goal_inset_applied) {
                        if (should_skip_goal(candidate, context) ||
                            is_same_as_last_goal(candidate, context.last_goal))
                        {
                            return false;
                        }
                        distance_m = grid_distance_in_meters(
                            robot_grid,
                            candidate,
                            resolution);
                        if (distance_m < min_goal_distance_m_ ||
                            !pass_map_candidate_constraints(
                                candidate,
                                environment.frontier_map,
                                &unknown_ratio) ||
                            !pass_safety_candidate_constraints(candidate, environment))
                        {
                            return false;
                        }
                    }
                }

                if (emitted_goals.count(candidate) > 0U) {
                    return false;
                }
                emitted_goals.insert(candidate);

                valid_candidates.push_back(FrontierCandidate{
                    candidate,
                    cluster.centroid,
                    cluster.cells.size(),
                    distance_m,
                    retry_count_of_goal(candidate, context),
                    compute_clearance_m(candidate, environment),
                    unknown_ratio,
                    cluster_index,
                    used_fallback,
                    goal_inset_applied,
                    false,
                    true,
                    0.0,
                    {}
                });
                return true;
            };

        cluster_generated_candidate |= append_candidate(cluster.centroid, false, true);

        const auto fallback = find_fallback_goal_in_cluster(
            cluster,
            robot_grid,
            resolution,
            environment.frontier_map,
            context);
        if (fallback.has_value()) {
            cluster_generated_candidate |= append_candidate(fallback.value(), true, true);
        }

        if (environment.frontier_map != nullptr && environment.frontier_map->isReady()) {
            double centroid_x = 0.0;
            double centroid_y = 0.0;
            double robot_x = 0.0;
            double robot_y = 0.0;
            environment.frontier_map->mapToWorld(
                cluster.centroid,
                centroid_x,
                centroid_y);
            environment.frontier_map->mapToWorld(robot_grid, robot_x, robot_y);

            const double direction_x = robot_x - centroid_x;
            const double direction_y = robot_y - centroid_y;
            const double direction_norm = std::hypot(direction_x, direction_y);
            if (direction_norm > 1e-6) {
                for (const auto retreat_distance_m : kRetreatDistancesM) {
                    GridCell candidate;
                    const double world_x =
                        centroid_x + direction_x / direction_norm * retreat_distance_m;
                    const double world_y =
                        centroid_y + direction_y / direction_norm * retreat_distance_m;
                    if (environment.frontier_map->worldToMap(
                            world_x,
                            world_y,
                            candidate))
                    {
                        cluster_generated_candidate |=
                            append_candidate(candidate, true, false);
                    }
                }
            }

            for (const auto sample_radius_m : kSampleRadiiM) {
                for (double angle_deg = 0.0;
                    angle_deg < 360.0;
                    angle_deg += kAngleStepDeg)
                {
                    const double angle_rad = angle_deg * kDegreesToRadians;
                    GridCell candidate;
                    const double world_x =
                        centroid_x + std::cos(angle_rad) * sample_radius_m;
                    const double world_y =
                        centroid_y + std::sin(angle_rad) * sample_radius_m;
                    if (environment.frontier_map->worldToMap(
                            world_x,
                            world_y,
                            candidate))
                    {
                        cluster_generated_candidate |=
                            append_candidate(candidate, true, false);
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
