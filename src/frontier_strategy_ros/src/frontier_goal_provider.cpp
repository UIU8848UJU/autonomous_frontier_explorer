#include "frontier_strategy_ros/frontier_goal_provider.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <utility>

#include "frontier_strategy_ros/adaptation/frontier_map_parameter_adapter.hpp"
#include "frontier_strategy_ros/geometry/footprint_collision_checker.hpp"

namespace frontier_strategy
{
namespace
{
constexpr double kResolutionComparisonEpsilon = 1e-9;
constexpr uint16_t kReasonOk = 0U;
constexpr uint16_t kReasonWaitingForMap = 1U;
constexpr uint16_t kReasonWaitingForRobotPose = 2U;
constexpr uint16_t kReasonCostmapNotReady = 3U;
constexpr uint16_t kReasonMapStale = 4U;
constexpr uint16_t kReasonNoFrontier = 5U;
constexpr uint16_t kReasonNoValidFrontier = 6U;
constexpr uint16_t kReasonAllFrontiersBlacklisted = 7U;

uint64_t map_fingerprint(const nav_msgs::msg::OccupancyGrid & map)
{
    uint64_t fingerprint = 1469598103934665603ULL;
    const auto mix = [&fingerprint](uint64_t value) {
        fingerprint ^= value;
        fingerprint *= 1099511628211ULL;
    };
    mix(map.info.width);
    mix(map.info.height);
    mix(std::hash<double>{}(map.info.resolution));
    mix(std::hash<double>{}(map.info.origin.position.x));
    mix(std::hash<double>{}(map.info.origin.position.y));
    for (const auto value : map.data) {
        mix(static_cast<uint8_t>(value));
    }
    return fingerprint;
}

FrontierStrategyPolicyConfig make_policy_config(const FrontierStrategyParams & params)
{
    FrontierStrategyPolicyConfig config;
    config.obstacle_search_radius_cells = params.runtime.obstacle_search_radius_cells;
    config.min_goal_distance_m = params.pruner.min_goal_distance_m;
    config.max_retry_count = params.selection.max_retry_count;
    config.max_cluster_retry_count = params.selection.max_cluster_retry_count;
    config.min_cluster_size = params.pruner.min_cluster_size;
    config.unknown_margin_cells = params.pruner.candidate_unknown_margin_cells;
    config.goal_inset_cells = params.pruner.candidate_goal_inset_cells;
    config.max_unknown_ratio = params.pruner.candidate_max_unknown_ratio;
    config.defer_small_clusters = params.selection.defer_small_clusters;
    config.small_cluster_size_threshold = params.selection.small_cluster_size_threshold;
    config.cleanup_enabled = params.runtime.cleanup_enabled;
    config.cleanup_min_cluster_size = params.pruner.cleanup_min_cluster_size;
    config.cleanup_min_goal_distance_m = params.pruner.cleanup_min_goal_distance_m;
    config.cleanup_goal_inset_cells = params.pruner.cleanup_goal_inset_cells;
    config.cleanup_trigger_no_candidate_cycles =
        params.runtime.cleanup_trigger_no_candidate_cycles;
    config.cleanup_trigger_only_small_clusters =
        params.runtime.cleanup_trigger_only_small_clusters;
    config.cleanup_max_unknown_ratio = params.pruner.cleanup_candidate_max_unknown_ratio;
    config.information_gain_sensor_range_m = params.pruner.enable_information_gain ?
        params.pruner.information_gain_sensor_range_m : 0.0;
    config.viewpoint_retreat_distances_m = params.pruner.viewpoint_retreat_distances_m;
    config.viewpoint_sample_radii_m = params.pruner.viewpoint_sample_radii_m;
    config.viewpoint_angle_step_deg = params.pruner.viewpoint_angle_step_deg;
    config.minimum_information_gain_m2 = params.pruner.minimum_information_gain_m2;
    config.require_reachable_goal = params.runtime.require_reachable_goal;
    config.scoring_weights = params.scorer.weights;
    config.scoring_weights.enable_information_gain_score =
        params.pruner.enable_information_gain;
    return config;
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

geometry_msgs::msg::Quaternion orientation_toward_centroid(
    const CostmapAdapter & costmap,
    const GridCell & goal,
    const GridCell & centroid)
{
    double goal_x = 0.0;
    double goal_y = 0.0;
    double centroid_x = 0.0;
    double centroid_y = 0.0;
    costmap.mapToWorld(
        static_cast<unsigned int>(goal.col),
        static_cast<unsigned int>(goal.row),
        goal_x,
        goal_y);
    costmap.mapToWorld(
        static_cast<unsigned int>(centroid.col),
        static_cast<unsigned int>(centroid.row),
        centroid_x,
        centroid_y);
    return quaternion_from_yaw(std::atan2(centroid_y - goal_y, centroid_x - goal_x));
}

}  // 命名空间

FrontierGoalProvider::FrontierGoalProvider(
    const rclcpp::Logger & logger,
    std::shared_ptr<IFrontierRanker> ranker)
: logger_(logger),
  map_costmap_(logger),
  global_costmap_(logger),
  ranker_(std::move(ranker)),
  policy_(make_policy_config(params_), ranker_)
{
}

void FrontierGoalProvider::configure(
    const FrontierStrategyParams & params,
    std::shared_ptr<const robot_geometry_core::IRobotGeometryProvider>
    robot_geometry_provider)
{
    if (!robot_geometry_provider) {
        throw std::invalid_argument("robot geometry provider must not be null");
    }
    robot_geometry_provider_ = std::move(robot_geometry_provider);
    base_params_ = params;
    params_ = base_params_;
    policy_map_resolution_.reset();
    if (map_costmap_.isReady()) {
        params_ = adapt_strategy_params_to_map_resolution(
            base_params_, map_costmap_.getResolution());
        policy_map_resolution_ = map_costmap_.getResolution();
    }
    policy_ = FrontierStrategyPolicy(make_policy_config(params_), ranker_);
}

void FrontierGoalProvider::refresh_policy_for_map_resolution(double map_resolution)
{
    if (!std::isfinite(map_resolution) || map_resolution <= 0.0) {
        return;
    }
    if (policy_map_resolution_.has_value() &&
        std::abs(policy_map_resolution_.value() - map_resolution) <=
        kResolutionComparisonEpsilon)
    {
        return;
    }

    const bool resolution_changed = policy_map_resolution_.has_value();
    params_ = adapt_strategy_params_to_map_resolution(base_params_, map_resolution);
    policy_ = FrontierStrategyPolicy(make_policy_config(params_), ranker_);
    policy_map_resolution_ = map_resolution;

    if (resolution_changed) {
        RCLCPP_WARN(
            logger_,
            "地图分辨率发生变化，已重建 frontier 策略并清空旧地图上的重试状态");
    }
    RCLCPP_INFO(
        logger_,
        "Frontier 参数已适配地图：resolution=%.4f obstacle_radius=%d "
        "min_cluster=%d small_cluster=%zu unknown_margin=%d goal_inset=%d cleanup_inset=%d",
        map_resolution,
        params_.runtime.obstacle_search_radius_cells,
        params_.runtime.min_frontier_cluster_size,
        params_.selection.small_cluster_size_threshold,
        params_.pruner.candidate_unknown_margin_cells,
        params_.pruner.candidate_goal_inset_cells,
        params_.pruner.cleanup_goal_inset_cells);
}

bool FrontierGoalProvider::update_map(
    const nav_msgs::msg::OccupancyGrid::SharedPtr & msg,
    const rclcpp::Time & stamp)
{
    if (!msg) {
        return false;
    }

    map_msg_ = msg;
    const bool updated = map_costmap_.updateFromOccupancyGrid(*msg);
    if (updated) {
        refresh_policy_for_map_resolution(msg->info.resolution);
        const auto fingerprint = map_fingerprint(*msg);
        if (!has_map_fingerprint_ || fingerprint != map_fingerprint_) {
            map_fingerprint_ = fingerprint;
            ++map_revision_;
            stable_no_frontier_cycles_ = 0;
            // 地图发生实质变化后，旧地图上的失败目标允许重新评估。
            policy_.clear_blacklist();
            consecutive_frontier_failures_ = 0U;
        }
        has_map_fingerprint_ = true;
        last_map_update_time_ = stamp;
    }
    return updated;
}

bool FrontierGoalProvider::update_global_costmap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr & msg)
{
    if (!msg) {
        return false;
    }
    return global_costmap_.updateFromOccupancyGrid(*msg);
}

void FrontierGoalProvider::update_robot_pose(const geometry_msgs::msg::PoseStamped & pose)
{
    robot_pose_ = pose;
}

void FrontierGoalProvider::clear_robot_pose()
{
    robot_pose_.reset();
}

void FrontierGoalProvider::set_reachability_checker(
    const std::shared_ptr<FrontierReachabilityChecker> & checker)
{
    reachability_checker_ = checker;
}

bool FrontierGoalProvider::update_robot_grid_position()
{
    if (!map_msg_ || !robot_pose_.has_value()) {
        RCLCPP_WARN(logger_, "map or robot pose is not available");
        return false;
    }

    if (robot_pose_->header.frame_id != params_.runtime.global_frame) {
        RCLCPP_WARN(
            logger_,
            "Robot pose frame mismatch: expected=%s actual=%s",
            params_.runtime.global_frame.c_str(),
            robot_pose_->header.frame_id.c_str());
        return false;
    }

    const double robot_x = robot_pose_->pose.position.x;
    const double robot_y = robot_pose_->pose.position.y;

    unsigned int col = 0U;
    unsigned int row = 0U;
    if (!map_costmap_.worldToMap(robot_x, robot_y, col, row)) {
        RCLCPP_WARN(logger_, "Robot grid position out of map bounds.");
        return false;
    }

    robot_grid_ = GridCell{static_cast<int>(row), static_cast<int>(col)};
    return true;
}

FrontierPruningEnvironment FrontierGoalProvider::make_pruning_environment(
    const CostmapAdapter & frontier_costmap,
    const CostmapAdapter * safety_costmap) const
{
    FrontierPruningEnvironment environment;
    robot_geometry_core::Footprint coarse_footprint;
    if (robot_geometry_provider_) {
        const auto geometry = robot_geometry_provider_->collisionEnvelope();
        coarse_footprint = robot_geometry_core::makeConservativeCircularFootprint(
            geometry.circumscribed_radius);
    }
    const auto coarse_ros_footprint =
        FootprintCollisionChecker::toRosFootprint(coarse_footprint);
    environment.frontier_map = &frontier_costmap.gridMap();
    environment.safety_check =
        [this, &frontier_costmap, safety_costmap, coarse_ros_footprint](
        const GridCell & cell) {
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
            if (!safety_costmap->worldToMap(world_x, world_y, safety_col, safety_row)) {
                return false;
            }

            const auto cost = safety_costmap->getCost(safety_col, safety_row);
            if (cost == nav2_costmap_2d::NO_INFORMATION ||
                cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                return false;
            }

            FootprintCollisionCheckerConfig footprint_config;
            footprint_config.enabled = params_.pruner.enable_footprint_filter;
            footprint_config.allow_unknown = params_.pruner.allow_unknown_footprint;
            footprint_config.cost_threshold = static_cast<unsigned char>(std::clamp(
                params_.pruner.footprint_cost_threshold, 1, 255));
            footprint_config.footprint = coarse_ros_footprint;
            const auto footprint_result = FootprintCollisionChecker::checkWorldPoint(
                *safety_costmap,
                world_x,
                world_y,
                0.0,
                footprint_config);
            if (!footprint_result.valid) {
                RCLCPP_DEBUG(
                    logger_,
                    "候选点未通过 footprint 过滤：row=%d col=%d reason=%s max_cost=%.1f",
                    cell.row,
                    cell.col,
                    footprint_result.reason.c_str(),
                    footprint_result.max_cost);
                return false;
            }
            return true;
        };

    environment.clearance_query =
        [this, &frontier_costmap, safety_costmap](const GridCell & cell) -> std::optional<double> {
            const CostmapAdapter * query_costmap =
                safety_costmap != nullptr ? safety_costmap : &frontier_costmap;
            if (!frontier_costmap.inBounds(cell.col, cell.row)) {
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
                if (!query_costmap->worldToMap(world_x, world_y, query_col, query_row)) {
                    return 0.0;
                }
            }

            const auto clearance = query_costmap->distanceToNearestObstacle(
                query_col,
                query_row,
                params_.pruner.candidate_unknown_margin_cells);
            return clearance.has_value() ? clearance : std::optional<double>(
                static_cast<double>(params_.pruner.candidate_unknown_margin_cells) *
                query_costmap->getResolution());
        };
    return environment;
}

FrontierCandidatesResult FrontierGoalProvider::compute_frontier_candidates(
    const rclcpp::Time & now,
    std::size_t max_candidates)
{
    const auto started = std::chrono::steady_clock::now();
    FrontierCandidatesResult result;
    result.blacklist_count = static_cast<uint32_t>(policy_.blacklisted_goals().size());
    result.visualization.blacklisted_goals = policy_.blacklisted_goals();

    if (!map_msg_) {
        RCLCPP_WARN(logger_, "No map data available.");
        result.reason_code = kReasonWaitingForMap;
        result.reason_text = "WAITING_FOR_MAP";
        result.recoverable = true;
        result.state = ExplorationStatus::RUNNING;
        result.state_detail = result.reason_text;
        return result;
    }

    if (last_map_update_time_.nanoseconds() > 0) {
        const auto elapsed = now - last_map_update_time_;
        if (elapsed > rclcpp::Duration(params_.runtime.map_stale_timeout)) {
            RCLCPP_WARN(
                logger_,
                "Map has not updated for %.2f seconds, marking STUCK.",
                elapsed.seconds());
            result.reason_code = kReasonMapStale;
            result.reason_text = "WAITING_FOR_MAP_UPDATE";
            result.recoverable = true;
            result.state = ExplorationStatus::STUCK;
            result.state_detail = "map_stale";
            result.visualization.clear_candidate_markers = true;
            return result;
        }
    }

    if (!update_robot_grid_position()) {
        RCLCPP_WARN(logger_, "Robot grid position unavailable.");
        result.reason_code = kReasonWaitingForRobotPose;
        result.reason_text = "WAITING_FOR_TF";
        result.recoverable = true;
        result.state = ExplorationStatus::RUNNING;
        result.state_detail = result.reason_text;
        return result;
    }
    result.visualization.robot_grid = robot_grid_;

    if (!map_costmap_.isReady()) {
        RCLCPP_WARN(logger_, "Costmap adapter is not ready.");
        result.reason_code = kReasonCostmapNotReady;
        result.reason_text = "COSTMAP_NOT_READY";
        result.recoverable = true;
        result.state = ExplorationStatus::RUNNING;
        result.state_detail = result.reason_text;
        return result;
    }

    const CostmapAdapter * safety_costmap =
        (params_.runtime.use_global_costmap_for_safety && global_costmap_.isReady()) ?
        &global_costmap_ : &map_costmap_;
    const auto map_frame = map_msg_->header.frame_id.empty() ?
        params_.runtime.global_frame : map_msg_->header.frame_id;

    std::size_t reachability_checks = 0U;
    auto reachability_check =
        [this, &reachability_checks, &map_frame](FrontierCandidate & candidate) {
            FrontierReachabilityResult unchecked;
            if (!params_.runtime.enable_reachability_filter || !reachability_checker_) {
                return unchecked;
            }
            if (!robot_pose_.has_value()) {
                unchecked.checked = true;
                unchecked.reachable = false;
                unchecked.reason = "robot_pose_unavailable";
                return unchecked;
            }
            if (reachability_checks >=
                static_cast<std::size_t>(params_.runtime.max_reachability_checks))
            {
                unchecked.checked = false;
                unchecked.reachable = true;
                unchecked.reason = "reachability_check_limit";
                return unchecked;
            }

            geometry_msgs::msg::PoseStamped goal_pose;
            goal_pose.header.frame_id = map_frame;
            goal_pose.header.stamp = robot_pose_->header.stamp;
            map_costmap_.mapToWorld(
                static_cast<unsigned int>(candidate.goal.col),
                static_cast<unsigned int>(candidate.goal.row),
                goal_pose.pose.position.x,
                goal_pose.pose.position.y);
            goal_pose.pose.position.z = 0.0;
            goal_pose.pose.orientation.w = 1.0;

            ++reachability_checks;
            return reachability_checker_->check(robot_pose_.value(), goal_pose);
        };

    const auto evaluation = policy_.evaluate(
        map_costmap_.gridMap(),
        robot_grid_.value(),
        make_pruning_environment(map_costmap_, safety_costmap),
        reachability_check);
    result.diagnostics = evaluation.diagnostics;
    result.cleanup_mode = evaluation.cleanup_mode;
    result.detection_ms = evaluation.detection_ms;
    result.pruning_ms = evaluation.pruning_ms;
    result.ranking_ms = evaluation.ranking_ms;
    result.map_revision = map_revision_;
    result.total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    RCLCPP_DEBUG(
        logger_,
        "Frontier decision timing: detection=%.3fms pruning=%.3fms ranking=%.3fms total=%.3fms "
        "raw_cells=%zu clusters=%zu candidates=%zu",
        result.detection_ms,
        result.pruning_ms,
        result.ranking_ms,
        result.total_ms,
        result.diagnostics.raw_frontier_cells,
        result.diagnostics.raw_clusters,
        result.diagnostics.generated_candidates);
    const auto & clusters = evaluation.clusters;
    const auto & scored_candidates = evaluation.scored_candidates;
    result.raw_frontier_count = static_cast<uint32_t>(evaluation.diagnostics.raw_frontier_cells);
    result.visualization.raw_clusters = clusters;

    RCLCPP_INFO(
        logger_,
        "检测 frontier：raw_cells=%zu，raw_clusters=%zu，min_cluster_size=%zu",
        evaluation.diagnostics.raw_frontier_cells,
        clusters.size(),
        params_.pruner.min_cluster_size);

    if (evaluation.clusters.empty()) {
        if (no_frontier_revision_ != map_revision_) {
            no_frontier_revision_ = map_revision_;
            stable_no_frontier_cycles_ = 0;
        }
        ++stable_no_frontier_cycles_;
        result.stable_no_frontier_cycles = stable_no_frontier_cycles_;
        if (stable_no_frontier_cycles_ < params_.runtime.stable_no_frontier_cycles) {
            result.reason_code = kReasonNoFrontier;
            result.reason_text = "WAITING_FOR_STABLE_NO_FRONTIER";
            result.exploration_complete = false;
            result.recoverable = true;
            result.state = ExplorationStatus::RUNNING;
            result.state_detail = result.reason_text;
            result.visualization.clear_candidate_markers = true;
            return result;
        }
        result.reason_code = kReasonNoFrontier;
        result.reason_text = "NO_FRONTIER_FOUND";
        result.exploration_complete = true;
        result.recoverable = false;
        result.state = ExplorationStatus::COMPLETED;
        result.state_detail = result.reason_text;
        result.visualization.clear_candidate_markers = true;
        return result;
    }

    // 重新看到 frontier 后，上一轮无 frontier 的稳定计数失效。
    stable_no_frontier_cycles_ = 0;
    no_frontier_revision_ = map_revision_;

    result.candidate_count = static_cast<uint32_t>(scored_candidates.size());
    result.blacklist_count = static_cast<uint32_t>(policy_.blacklisted_goals().size());
    result.visualization.scored_candidates = scored_candidates;
    result.visualization.blacklisted_goals = policy_.blacklisted_goals();
    result.visualization.candidates.reserve(scored_candidates.size());
    for (const auto & scored : scored_candidates) {
        result.visualization.candidates.push_back(scored.candidate);
    }

    if (scored_candidates.empty()) {
        result.visualization.clear_candidate_markers = true;
        result.visualization.rejected_clusters = clusters;
        ++consecutive_frontier_failures_;
        result.reason_code = policy_.blacklisted_goals().empty() ?
            kReasonNoValidFrontier : kReasonAllFrontiersBlacklisted;
        result.reason_text = policy_.blacklisted_goals().empty() ?
            "NO_VALID_FRONTIER" : "ALL_FRONTIERS_BLACKLISTED";
        result.exploration_complete = false;
        const bool retry_limit_reached = consecutive_frontier_failures_ >=
            static_cast<std::size_t>(std::max(1, params_.runtime.max_frontier_failures));
        result.recoverable = !retry_limit_reached;
        result.state = retry_limit_reached ? ExplorationStatus::STUCK : ExplorationStatus::RUNNING;
        result.state_detail = retry_limit_reached ?
            result.reason_text : "WAITING_FOR_FRONTIER_RETRY";
        return result;
    }

    consecutive_frontier_failures_ = 0U;
    result.success = true;
    result.reason_code = kReasonOk;
    result.reason_text = "FRONTIER_CANDIDATES_READY";
    result.state = ExplorationStatus::RUNNING;
    result.state_detail = result.reason_text;
    result.candidates.reserve(max_candidates == 0U ?
        scored_candidates.size() : std::min(max_candidates, scored_candidates.size()));

    const std::size_t limit = max_candidates == 0U ?
        scored_candidates.size() : std::min(max_candidates, scored_candidates.size());
    for (std::size_t index = 0; index < limit; ++index) {
        const auto & scored = scored_candidates[index];
        robot_interfaces::msg::FrontierCandidate candidate_result;
        candidate_result.goal.header.frame_id = map_frame;
        candidate_result.goal.header.stamp = now;
        map_costmap_.mapToWorld(
            static_cast<unsigned int>(scored.candidate.goal.col),
            static_cast<unsigned int>(scored.candidate.goal.row),
            candidate_result.goal.pose.position.x,
        candidate_result.goal.pose.position.y);
        candidate_result.goal.pose.position.z = 0.0;
        candidate_result.goal.pose.orientation = orientation_toward_centroid(
            map_costmap_,
            scored.candidate.goal,
            scored.candidate.cluster_centroid);
        candidate_result.goal_row = scored.candidate.goal.row;
        candidate_result.goal_col = scored.candidate.goal.col;
        candidate_result.cluster_centroid_row = scored.candidate.cluster_centroid.row;
        candidate_result.cluster_centroid_col = scored.candidate.cluster_centroid.col;
        candidate_result.source_cluster_index = static_cast<uint32_t>(
            scored.candidate.source_cluster_index);
        candidate_result.score = static_cast<float>(scored.total_score);
        candidate_result.distance_m = static_cast<float>(scored.candidate.distance_m);
        candidate_result.clearance_m = static_cast<float>(scored.candidate.clearance_m);
        candidate_result.unknown_ratio = static_cast<float>(scored.candidate.unknown_ratio);
        candidate_result.information_gain = static_cast<float>(scored.candidate.information_gain);
        candidate_result.cluster_size = static_cast<uint32_t>(scored.candidate.cluster_size);
        candidate_result.retry_count = static_cast<uint32_t>(
            std::max(0, scored.candidate.retry_count));
        candidate_result.used_fallback = scored.candidate.used_fallback;
        candidate_result.goal_inset_applied = scored.candidate.goal_inset_applied;
        candidate_result.reachability_checked = scored.candidate.reachability_checked;
        candidate_result.reachable = scored.candidate.reachable;
        candidate_result.path_length_m = static_cast<float>(scored.candidate.path_length_m);
        candidate_result.reachability_reason = scored.candidate.reachability_reason;
        candidate_result.information_gain_valid = scored.candidate.information_gain_valid;
        result.candidates.push_back(candidate_result);
    }

    return result;
}

FrontierFailureResult FrontierGoalProvider::mark_frontier_failed(
    const geometry_msgs::msg::Point & failed_goal)
{
    FrontierFailureResult result;
    result.blacklisted_goals = policy_.blacklisted_goals();

    if (!map_costmap_.isReady()) {
        result.success = false;
        result.message = "map costmap is not ready";
        return result;
    }

    unsigned int col = 0U;
    unsigned int row = 0U;
    if (!map_costmap_.worldToMap(failed_goal.x, failed_goal.y, col, row)) {
        result.success = false;
        result.message = "failed goal is outside map";
        result.state = ExplorationStatus::STUCK;
        result.state_detail = "MARK_FAILED_OUT_OF_MAP";
        return result;
    }

    const GridCell failed_cell{static_cast<int>(row), static_cast<int>(col)};
    policy_.mark_goal_failed(failed_cell);
    result.retry_count = static_cast<uint32_t>(policy_.retry_count_for_goal(failed_cell));
    result.blacklisted = policy_.is_goal_blacklisted(failed_cell);
    result.success = true;
    result.message = result.blacklisted ? "frontier blacklisted" : "frontier failure recorded";
    result.state = ExplorationStatus::RUNNING;
    result.state_detail = "FRONTIER_FAILURE_RECORDED";
    result.blacklisted_goals = policy_.blacklisted_goals();
    return result;
}

std::size_t FrontierGoalProvider::clear_blacklist()
{
    return policy_.clear_blacklist();
}

const CostmapAdapter & FrontierGoalProvider::map_costmap() const
{
    return map_costmap_;
}

std::vector<GridCell> FrontierGoalProvider::blacklisted_goals() const
{
    return policy_.blacklisted_goals();
}

}  // 命名空间 frontier_strategy
