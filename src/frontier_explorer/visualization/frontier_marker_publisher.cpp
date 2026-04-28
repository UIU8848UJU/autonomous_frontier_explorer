#include "frontier_marker_publisher.hpp"

#include <algorithm>
#include <sstream>

#include "visualization_msgs/msg/marker.hpp"

namespace frontier_explorer
{

namespace
{
constexpr double kPointScale = 0.05;
constexpr double kSphereScale = 0.15;
constexpr double kTextScale = 0.16;
constexpr double kTextHeightOffset = 0.28;
constexpr double kArrowLength = 0.4;
constexpr double kArrowShaft = 0.08;
constexpr double kMarkerAlpha = 0.9;
constexpr std::size_t kMaxScoredTextMarkers = 5U;
}  // namespace

FrontierMarkerPublisher::FrontierMarkerPublisher(
    rclcpp::Node * node,
    const rclcpp::Logger & logger,
    const std::string & frame_id)
: node_(node),
  logger_(rclcpp::Logger(logger).get_child("markers")),
  frame_id_(frame_id)
{
    if (node_ == nullptr) {
        RCLCPP_WARN(logger_, "FrontierMarkerPublisher constructed with null node.");
        return;
    }

    raw_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/frontier/raw_markers", rclcpp::QoS(1).transient_local());
    candidate_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/frontier/candidate_markers", rclcpp::QoS(1).transient_local());
    rejected_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/frontier/rejected_markers", rclcpp::QoS(1).transient_local());
    scored_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/frontier/scored_markers", rclcpp::QoS(1).transient_local());
    selected_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/frontier/selected_marker", rclcpp::QoS(1).transient_local());
    blacklist_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/frontier/blacklist_markers", rclcpp::QoS(1).transient_local());
}

void FrontierMarkerPublisher::clearAll() const
{
    publishDeleteAll(raw_pub_);
    publishDeleteAll(candidate_pub_);
    publishDeleteAll(rejected_pub_);
    publishDeleteAll(scored_pub_);
    publishDeleteAll(selected_pub_);
    publishDeleteAll(blacklist_pub_);
}

void FrontierMarkerPublisher::publishRawFrontiers(
    const std::vector<FrontierCluster> & clusters,
    const CostmapAdapter & costmap) const
{
    if (!raw_pub_ || !costmap.isReady()) {
        return;
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(makeDeleteAllMarker());

    auto marker = makeBaseMarker(
        "raw_frontiers",
        1,
        visualization_msgs::msg::Marker::POINTS);
    marker.scale.x = kPointScale;
    marker.scale.y = kPointScale;
    marker.color.r = 0.1F;
    marker.color.g = 0.35F;
    marker.color.b = 1.0F;
    marker.color.a = kMarkerAlpha;

    for (const auto & cluster : clusters) {
        for (const auto & cell : cluster.cells) {
            geometry_msgs::msg::Point point;
            if (cellToWorldPoint(cell, costmap, point)) {
                marker.points.push_back(point);
            }
        }
    }

    array.markers.push_back(marker);
    raw_pub_->publish(array);
}

void FrontierMarkerPublisher::publishCandidates(
    const std::vector<FrontierCandidate> & candidates,
    const CostmapAdapter & costmap) const
{
    if (!candidate_pub_ || !costmap.isReady()) {
        return;
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(makeDeleteAllMarker());

    int id = 1;
    for (const auto & candidate : candidates) {
        geometry_msgs::msg::Point point;
        if (!cellToWorldPoint(candidate.goal, costmap, point)) {
            continue;
        }

        auto marker = makeBaseMarker(
            "candidates",
            id++,
            visualization_msgs::msg::Marker::SPHERE);
        marker.pose.position = point;
        marker.scale.x = kSphereScale;
        marker.scale.y = kSphereScale;
        marker.scale.z = kSphereScale;
        marker.color.r = 0.0F;
        marker.color.g = 0.9F;
        marker.color.b = 1.0F;
        marker.color.a = kMarkerAlpha;
        array.markers.push_back(marker);
    }

    candidate_pub_->publish(array);
}

void FrontierMarkerPublisher::publishRejectedCandidates(
    const std::vector<FrontierCandidate> & rejected_candidates,
    const CostmapAdapter & costmap) const
{
    if (!rejected_pub_ || !costmap.isReady()) {
        return;
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(makeDeleteAllMarker());

    int id = 1;
    for (const auto & candidate : rejected_candidates) {
        geometry_msgs::msg::Point point;
        if (!cellToWorldPoint(candidate.goal, costmap, point)) {
            continue;
        }

        auto marker = makeBaseMarker(
            "rejected_candidates",
            id++,
            visualization_msgs::msg::Marker::SPHERE);
        marker.pose.position = point;
        marker.scale.x = kSphereScale;
        marker.scale.y = kSphereScale;
        marker.scale.z = kSphereScale;
        marker.color.r = 1.0F;
        marker.color.g = 0.0F;
        marker.color.b = 0.0F;
        marker.color.a = kMarkerAlpha;
        array.markers.push_back(marker);
    }

    rejected_pub_->publish(array);
}

void FrontierMarkerPublisher::publishRejectedFrontiers(
    const std::vector<FrontierCluster> & clusters,
    const CostmapAdapter & costmap) const
{
    if (!rejected_pub_ || !costmap.isReady()) {
        return;
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(makeDeleteAllMarker());

    auto marker = makeBaseMarker(
        "rejected_frontiers",
        1,
        visualization_msgs::msg::Marker::POINTS);
    marker.scale.x = kPointScale;
    marker.scale.y = kPointScale;
    marker.color.r = 1.0F;
    marker.color.g = 0.0F;
    marker.color.b = 0.0F;
    marker.color.a = kMarkerAlpha;

    for (const auto & cluster : clusters) {
        for (const auto & cell : cluster.cells) {
            geometry_msgs::msg::Point point;
            if (cellToWorldPoint(cell, costmap, point)) {
                marker.points.push_back(point);
            }
        }
    }

    array.markers.push_back(marker);
    rejected_pub_->publish(array);
}

void FrontierMarkerPublisher::publishScoredCandidates(
    const std::vector<ScoredFrontierCandidate> & scored_candidates,
    const CostmapAdapter & costmap) const
{
    if (!scored_pub_ || !costmap.isReady()) {
        return;
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(makeDeleteAllMarker());

    std::vector<ScoredFrontierCandidate> sorted = scored_candidates;
    std::sort(
        sorted.begin(),
        sorted.end(),
        [](const ScoredFrontierCandidate & lhs, const ScoredFrontierCandidate & rhs) {
            return lhs.total_score > rhs.total_score;
        });

    int id = 1;
    const std::size_t count = std::min(kMaxScoredTextMarkers, sorted.size());
    for (std::size_t index = 0; index < count; ++index) {
        const auto & scored = sorted[index];
        geometry_msgs::msg::Point point;
        if (!cellToWorldPoint(scored.candidate.goal, costmap, point)) {
            continue;
        }

        auto marker = makeBaseMarker(
            "scored_candidates",
            id++,
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        marker.pose.position = point;
        marker.pose.position.z += kTextHeightOffset;
        marker.scale.z = kTextScale;

        if (scored.candidate.retry_count >= 2) {
            marker.color.r = 0.7F;
            marker.color.g = 0.2F;
            marker.color.b = 1.0F;
        } else {
            marker.color.r = 1.0F;
            marker.color.g = 0.95F;
            marker.color.b = 0.1F;
        }
        marker.color.a = 1.0F;

        std::ostringstream text;
        text.precision(2);
        text << std::fixed
             << "#" << (index + 1)
             << " score " << scored.total_score;
        marker.text = text.str();

        array.markers.push_back(marker);
    }

    scored_pub_->publish(array);
}

void FrontierMarkerPublisher::publishSelectedGoal(
    const GridCell & goal,
    const GridCell & robot,
    const CostmapAdapter & costmap) const
{
    if (!selected_pub_ || !costmap.isReady()) {
        return;
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(makeDeleteAllMarker());

    geometry_msgs::msg::Point goal_point;
    geometry_msgs::msg::Point robot_point;
    if (cellToWorldPoint(goal, costmap, goal_point) &&
        cellToWorldPoint(robot, costmap, robot_point))
    {
        auto marker = makeBaseMarker(
            "selected_goal",
            1,
            visualization_msgs::msg::Marker::ARROW);
        marker.points.push_back(robot_point);
        marker.points.push_back(goal_point);
        marker.scale.x = kArrowShaft;
        marker.scale.y = kArrowShaft * 2.0;
        marker.scale.z = kArrowLength * 0.45;
        marker.color.r = 0.0F;
        marker.color.g = 1.0F;
        marker.color.b = 0.0F;
        marker.color.a = 1.0F;
        array.markers.push_back(marker);
    }

    selected_pub_->publish(array);
}

void FrontierMarkerPublisher::publishBlacklist(
    const std::vector<GridCell> & blacklist,
    const CostmapAdapter & costmap) const
{
    if (!blacklist_pub_ || !costmap.isReady()) {
        return;
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(makeDeleteAllMarker());

    int id = 1;
    for (const auto & cell : blacklist) {
        geometry_msgs::msg::Point point;
        if (!cellToWorldPoint(cell, costmap, point)) {
            continue;
        }

        auto marker = makeBaseMarker(
            "blacklist",
            id++,
            visualization_msgs::msg::Marker::SPHERE);
        marker.pose.position = point;
        marker.scale.x = kSphereScale;
        marker.scale.y = kSphereScale;
        marker.scale.z = kSphereScale;
        marker.color.r = 1.0F;
        marker.color.g = 0.0F;
        marker.color.b = 0.0F;
        marker.color.a = kMarkerAlpha;
        array.markers.push_back(marker);
    }

    blacklist_pub_->publish(array);
}

visualization_msgs::msg::Marker FrontierMarkerPublisher::makeDeleteAllMarker() const
{
    auto marker = makeBaseMarker("clear", 0, visualization_msgs::msg::Marker::CUBE);
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
}

void FrontierMarkerPublisher::publishDeleteAll(
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher) const
{
    if (!publisher) {
        return;
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(makeDeleteAllMarker());
    publisher->publish(array);
}

visualization_msgs::msg::Marker FrontierMarkerPublisher::makeBaseMarker(
    const std::string & marker_namespace,
    int id,
    int type) const
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_ != nullptr ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    marker.ns = marker_namespace;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
}

bool FrontierMarkerPublisher::cellToWorldPoint(
    const GridCell & cell,
    const CostmapAdapter & costmap,
    geometry_msgs::msg::Point & point) const
{
    if (!costmap.inBounds(cell.col, cell.row)) {
        return false;
    }

    costmap.mapToWorld(
        static_cast<unsigned int>(cell.col),
        static_cast<unsigned int>(cell.row),
        point.x,
        point.y);
    point.z = 0.0;
    return true;
}

}  // namespace frontier_explorer
