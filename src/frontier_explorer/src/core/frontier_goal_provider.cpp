#include "core/frontier_goal_provider.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace frontier_explorer
{
namespace
{
constexpr double kEpsilon = 1e-6;
constexpr uint16_t kReasonOk = 0U;
constexpr uint16_t kReasonWaitingForMap = 1U;
constexpr uint16_t kReasonWaitingForRobotPose = 2U;
constexpr uint16_t kReasonCostmapNotReady = 3U;
constexpr uint16_t kReasonMapStale = 4U;
constexpr uint16_t kReasonNoFrontier = 5U;
constexpr uint16_t kReasonNoValidFrontier = 6U;
constexpr uint16_t kReasonAllFrontiersBlacklisted = 7U;

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

double distance_to_map_edge_m(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridCell & cell)
{
    if (map.info.width == 0 || map.info.height == 0) {
        return std::numeric_limits<double>::infinity();
    }

    const int max_row = static_cast<int>(map.info.height) - 1;
    const int max_col = static_cast<int>(map.info.width) - 1;

    const int dist_top = std::max(0, cell.row);
    const int dist_bottom = std::max(0, max_row - cell.row);
    const int dist_left = std::max(0, cell.col);
    const int dist_right = std::max(0, max_col - cell.col);

    const int min_cells = std::min(std::min(dist_top, dist_bottom), std::min(dist_left, dist_right));
    if (min_cells < 0) {
        return 0.0;
    }

    return static_cast<double>(min_cells) * map.info.resolution;
}
}  // namespace

FrontierGoalProvider::FrontierGoalProvider(const rclcpp::Logger & logger)
: logger_(logger),
  map_costmap_(logger),
  global_costmap_(logger),
  detector_(params_.runtime.obstacle_search_radius_cells, logger),
  selector_(
      params_.pruner.min_goal_distance_m,
      params_.selection.max_retry_count,
      params_.scorer.weights,
      params_.pruner.min_cluster_size,
      params_.selection.max_cluster_retry_count,
      params_.pruner.candidate_unknown_margin_cells,
      params_.pruner.candidate_goal_inset_cells,
      params_.pruner.candidate_max_unknown_ratio,
      params_.selection.defer_small_clusters,
      params_.selection.small_cluster_size_threshold,
      params_.runtime.require_reachable_goal,
      logger)
{
}

void FrontierGoalProvider::configure(const FrontierExplorerParams & params)
{
    params_ = params;
    detector_ = FrontierDetector(params_.runtime.obstacle_search_radius_cells, logger_);
    selector_ = FrontierSelector(
        params_.pruner.min_goal_distance_m,
        params_.selection.max_retry_count,
        params_.scorer.weights,
        params_.pruner.min_cluster_size,
        params_.selection.max_cluster_retry_count,
        params_.pruner.candidate_unknown_margin_cells,
        params_.pruner.candidate_goal_inset_cells,
        params_.pruner.candidate_max_unknown_ratio,
        params_.selection.defer_small_clusters,
        params_.selection.small_cluster_size_threshold,
        params_.runtime.require_reachable_goal,
        logger_
    );
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

bool FrontierGoalProvider::near_map_edge(const GridCell & cell, double tolerance_m) const
{
    if (!map_msg_ || tolerance_m <= kEpsilon) {
        return false;
    }

    const double distance = distance_to_map_edge_m(*map_msg_, cell);
    return std::isfinite(distance) && distance <= tolerance_m;
}

FrontierCandidatesResult FrontierGoalProvider::compute_frontier_candidates(
    const rclcpp::Time & now,
    std::size_t max_candidates)
{
    FrontierCandidatesResult result;
    result.blacklist_count = static_cast<uint32_t>(selector_.blacklisted_goals().size());
    result.visualization.blacklisted_goals = selector_.blacklisted_goals();

    if (!map_msg_) {
        RCLCPP_WARN(logger_, "No map data available.");
        result.reason_code = kReasonWaitingForMap;
        result.reason_text = "WAITING_FOR_MAP";
        result.recoverable = true;
        result.state = ExplorationState::RUNNING;
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
            result.state = ExplorationState::STUCK;
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
        result.state = ExplorationState::RUNNING;
        result.state_detail = result.reason_text;
        return result;
    }
    result.visualization.robot_grid = robot_grid_;

    if (!map_costmap_.isReady()) {
        RCLCPP_WARN(logger_, "Costmap adapter is not ready.");
        result.reason_code = kReasonCostmapNotReady;
        result.reason_text = "COSTMAP_NOT_READY";
        result.recoverable = true;
        result.state = ExplorationState::RUNNING;
        result.state_detail = result.reason_text;
        return result;
    }

    const auto frontier_cells = detector_.detect_frontier_cells(map_costmap_);
    const auto clusters = detector_.cluster_frontiers(map_costmap_, frontier_cells);
    result.raw_frontier_count = static_cast<uint32_t>(clusters.size());
    result.visualization.raw_clusters = clusters;

    RCLCPP_INFO(
        logger_,
        "Detected frontier cells: %zu, raw clusters: %zu, min_cluster_size=%zu",
        frontier_cells.size(),
        clusters.size(),
        params_.pruner.min_cluster_size);

    if (clusters.empty()) {
        result.reason_code = kReasonNoFrontier;
        result.reason_text = "NO_FRONTIER_FOUND";
        result.exploration_complete = true;
        result.recoverable = false;
        result.state = ExplorationState::COMPLETED;
        result.state_detail = result.reason_text;
        result.visualization.clear_candidate_markers = true;
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

    const auto scored_candidates = selector_.rank_frontier_candidates(
        clusters,
        robot_grid_.value(),
        map_costmap_.getResolution(),
        map_costmap_,
        safety_costmap,
        reachability_check);

    result.candidate_count = static_cast<uint32_t>(scored_candidates.size());
    result.blacklist_count = static_cast<uint32_t>(selector_.blacklisted_goals().size());
    result.visualization.scored_candidates = scored_candidates;
    result.visualization.blacklisted_goals = selector_.blacklisted_goals();
    result.visualization.candidates.reserve(scored_candidates.size());
    for (const auto & scored : scored_candidates) {
        result.visualization.candidates.push_back(scored.candidate);
    }

    if (scored_candidates.empty()) {
        result.visualization.clear_candidate_markers = true;
        result.visualization.rejected_clusters = clusters;
        ++consecutive_frontier_failures_;
        result.reason_code = selector_.blacklisted_goals().empty() ?
            kReasonNoValidFrontier : kReasonAllFrontiersBlacklisted;
        result.reason_text = selector_.blacklisted_goals().empty() ?
            "NO_VALID_FRONTIER" : "ALL_FRONTIERS_BLACKLISTED";
        result.exploration_complete = false;
        result.recoverable = false;
        result.state = ExplorationState::STUCK;
        result.state_detail = result.reason_text;
        return result;
    }

    consecutive_frontier_failures_ = 0U;
    result.success = true;
    result.reason_code = kReasonOk;
    result.reason_text = "FRONTIER_CANDIDATES_READY";
    result.state = ExplorationState::RUNNING;
    result.state_detail = result.reason_text;
    result.candidates.reserve(max_candidates == 0U ?
        scored_candidates.size() : std::min(max_candidates, scored_candidates.size()));

    const std::size_t limit = max_candidates == 0U ?
        scored_candidates.size() : std::min(max_candidates, scored_candidates.size());
    for (std::size_t index = 0; index < limit; ++index) {
        const auto & scored = scored_candidates[index];
        FrontierCandidateResult candidate_result;
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
        candidate_result.score = static_cast<float>(scored.total_score);
        candidate_result.distance_m = static_cast<float>(scored.candidate.distance_m);
        candidate_result.clearance_m = static_cast<float>(scored.candidate.clearance_m);
        candidate_result.unknown_ratio = static_cast<float>(scored.candidate.unknown_ratio);
        candidate_result.cluster_size = static_cast<uint32_t>(scored.candidate.cluster_size);
        candidate_result.retry_count = static_cast<uint32_t>(
            std::max(0, scored.candidate.retry_count));
        candidate_result.used_fallback = scored.candidate.used_fallback;
        candidate_result.goal_inset_applied = scored.candidate.goal_inset_applied;
        candidate_result.reachability_checked = scored.candidate.reachability_checked;
        candidate_result.reachable = scored.candidate.reachable;
        candidate_result.path_length_m = static_cast<float>(scored.candidate.path_length_m);
        candidate_result.goal_cell = scored.candidate.goal;
        result.candidates.push_back(candidate_result);
    }

    return result;
}

FrontierGoalResult FrontierGoalProvider::compute_next_frontier_goal(const rclcpp::Time & now)
{
    FrontierGoalResult result;
    result.goal.header.frame_id = params_.runtime.global_frame;
    result.goal.header.stamp = now;
    result.blacklist_count = static_cast<uint32_t>(selector_.blacklisted_goals().size());
    result.visualization.blacklisted_goals = selector_.blacklisted_goals();

    if (!map_msg_) {
        RCLCPP_WARN(logger_, "No map data available.");
        result.reason_code = kReasonWaitingForMap;
        result.reason_text = "WAITING_FOR_MAP";
        result.recoverable = true;
        result.state = ExplorationState::RUNNING;
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
            result.state = ExplorationState::STUCK;
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
        result.state = ExplorationState::RUNNING;
        result.state_detail = result.reason_text;
        return result;
    }
    result.visualization.robot_grid = robot_grid_;

    if (!map_costmap_.isReady()) {
        RCLCPP_WARN(logger_, "Costmap adapter is not ready.");
        result.reason_code = kReasonCostmapNotReady;
        result.reason_text = "COSTMAP_NOT_READY";
        result.recoverable = true;
        result.state = ExplorationState::RUNNING;
        result.state_detail = result.reason_text;
        return result;
    }

    const auto frontier_cells = detector_.detect_frontier_cells(map_costmap_);
    const auto clusters = detector_.cluster_frontiers(map_costmap_, frontier_cells);
    result.raw_frontier_count = static_cast<uint32_t>(clusters.size());
    result.visualization.raw_clusters = clusters;

    RCLCPP_INFO(
        logger_,
        "Detected frontier cells: %zu, raw clusters: %zu, min_cluster_size=%zu",
        frontier_cells.size(),
        clusters.size(),
        params_.pruner.min_cluster_size);

    if (clusters.empty()) {
        result.reason_code = kReasonNoFrontier;
        result.reason_text = "NO_FRONTIER_FOUND";
        result.exploration_complete = true;
        result.recoverable = false;
        result.state = ExplorationState::COMPLETED;
        result.state_detail = result.reason_text;
        result.visualization.clear_candidate_markers = true;
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

    const auto best_frontier = selector_.choose_best_frontier(
        clusters,
        robot_grid_.value(),
        map_costmap_.getResolution(),
        map_costmap_,
        safety_costmap,
        reachability_check);

    result.candidate_count = static_cast<uint32_t>(selector_.last_scored_candidates().size());
    result.blacklist_count = static_cast<uint32_t>(selector_.blacklisted_goals().size());
    result.visualization.scored_candidates = selector_.last_scored_candidates();
    result.visualization.blacklisted_goals = selector_.blacklisted_goals();
    result.visualization.candidates.reserve(selector_.last_scored_candidates().size());
    for (const auto & scored : selector_.last_scored_candidates()) {
        result.visualization.candidates.push_back(scored.candidate);
    }

    if (!best_frontier.has_value()) {
        result.visualization.clear_candidate_markers = true;
        result.visualization.rejected_clusters = clusters;
        RCLCPP_WARN(
            logger_,
            "No valid frontier selected. robot_grid=(%d, %d), clusters=%zu",
            robot_grid_->row,
            robot_grid_->col,
            clusters.size());
        ++consecutive_frontier_failures_;
        result.reason_code = selector_.blacklisted_goals().empty() ?
            kReasonNoValidFrontier : kReasonAllFrontiersBlacklisted;
        result.reason_text = selector_.blacklisted_goals().empty() ?
            "NO_VALID_FRONTIER" : "ALL_FRONTIERS_BLACKLISTED";
        result.exploration_complete = false;
        result.recoverable = false;
        result.state = ExplorationState::STUCK;
        result.state_detail = result.reason_text;

        if (consecutive_frontier_failures_ >=
            static_cast<std::size_t>(params_.runtime.max_frontier_failures))
        {
            const bool near_edge = robot_grid_.has_value() &&
                near_map_edge(robot_grid_.value(), params_.runtime.edge_tolerance_m);
            result.state_detail = near_edge ? "no_frontier_near_edge" : "no_valid_frontier";
            RCLCPP_WARN(
                logger_,
                "Frontier selection failed %zu times (near_edge=%s).",
                consecutive_frontier_failures_,
                near_edge ? "true" : "false");
            consecutive_frontier_failures_ = 0U;
        }
        return result;
    }

    consecutive_frontier_failures_ = 0U;
    result.visualization.selected_goal = best_frontier.value();
    const ScoredFrontierCandidate * selected_scored_candidate = nullptr;
    RCLCPP_INFO(
        logger_,
        "Chosen frontier: row=%d, col=%d, robot=(%d, %d)",
        best_frontier->row,
        best_frontier->col,
        robot_grid_->row,
        robot_grid_->col);

    auto & goal_pose = result.goal;
    goal_pose.header.frame_id = map_frame;
    goal_pose.header.stamp = now;
    map_costmap_.mapToWorld(
        static_cast<unsigned int>(best_frontier->col),
        static_cast<unsigned int>(best_frontier->row),
        goal_pose.pose.position.x,
        goal_pose.pose.position.y);
    goal_pose.pose.position.z = 0.0;
    result.success = true;
    result.goal_cell = best_frontier.value();
    result.reason_code = kReasonOk;
    result.reason_text = "FRONTIER_SELECTED";

    for (const auto & scored : selector_.last_scored_candidates()) {
        if (scored.candidate.goal == best_frontier.value()) {
            selected_scored_candidate = &scored;
            result.score = static_cast<float>(scored.total_score);
            result.distance_m = static_cast<float>(scored.candidate.distance_m);
            result.clearance_m = static_cast<float>(scored.candidate.clearance_m);
            break;
        }
    }
    if (selected_scored_candidate != nullptr) {
        goal_pose.pose.orientation = orientation_toward_centroid(
            map_costmap_,
            selected_scored_candidate->candidate.goal,
            selected_scored_candidate->candidate.cluster_centroid);
    } else {
        goal_pose.pose.orientation.w = 1.0;
    }

    selector_.set_last_goal(best_frontier.value());
    result.state = ExplorationState::RUNNING;
    result.state_detail = result.reason_text;
    return result;
}

FrontierFailureResult FrontierGoalProvider::mark_frontier_failed(
    const geometry_msgs::msg::Point & failed_goal)
{
    FrontierFailureResult result;
    result.blacklisted_goals = selector_.blacklisted_goals();

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
        result.state = ExplorationState::STUCK;
        result.state_detail = "MARK_FAILED_OUT_OF_MAP";
        return result;
    }

    const GridCell failed_cell{static_cast<int>(row), static_cast<int>(col)};
    selector_.mark_goal_failed(failed_cell);
    result.retry_count = static_cast<uint32_t>(selector_.retry_count_for_goal(failed_cell));
    result.blacklisted = selector_.is_goal_blacklisted(failed_cell);
    result.success = true;
    result.message = result.blacklisted ? "frontier blacklisted" : "frontier failure recorded";
    result.state = ExplorationState::RUNNING;
    result.state_detail = "FRONTIER_FAILURE_RECORDED";
    result.blacklisted_goals = selector_.blacklisted_goals();
    return result;
}

std::size_t FrontierGoalProvider::clear_blacklist()
{
    return selector_.clear_blacklist();
}

const CostmapAdapter & FrontierGoalProvider::map_costmap() const
{
    return map_costmap_;
}

std::vector<GridCell> FrontierGoalProvider::blacklisted_goals() const
{
    return selector_.blacklisted_goals();
}

}  // namespace frontier_explorer
