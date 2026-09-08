#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "frontier_explorer_ros/costmap/costmap_adapter.hpp"
#include "frontier_explorer_core/detector/frontier_detector.hpp"
#include "frontier_explorer_ros/frontier_goal_provider.hpp"
#include "frontier_explorer_ros/reachability/frontier_reachability_checker.hpp"
#include "frontier_explorer_core/scoring/frontier_scorer.hpp"
#include "frontier_explorer_core/selector/filters/frontier_pruner.hpp"
#include "frontier_explorer_ros/selector/frontier_selector.hpp"
#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_explorer
{
namespace
{

nav_msgs::msg::OccupancyGrid make_grid(
    unsigned int width,
    unsigned int height,
    float resolution,
    const std::vector<int8_t> & data)
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.resolution = resolution;
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.data = data;
    return grid;
}

nav_msgs::msg::OccupancyGrid make_filled_grid(
    unsigned int width,
    unsigned int height,
    float resolution,
    int8_t value)
{
    return make_grid(width, height, resolution, std::vector<int8_t>(width * height, value));
}

CostmapAdapter make_costmap(const nav_msgs::msg::OccupancyGrid & grid)
{
    CostmapAdapter adapter(rclcpp::get_logger("frontier_explorer_ros_test"));
    EXPECT_TRUE(adapter.updateFromOccupancyGrid(grid));
    return adapter;
}

FrontierCluster make_cluster(const std::vector<GridCell> & cells, GridCell centroid)
{
    FrontierCluster cluster;
    cluster.cells = cells;
    cluster.centroid = centroid;
    return cluster;
}

bool contains_cell(const std::vector<GridCell> & cells, const GridCell & target)
{
    return std::find(cells.begin(), cells.end(), target) != cells.end();
}

}  // namespace

TEST(CostmapAdapterTest, ConvertsOccupancyGridAndCoordinates)
{
    const auto grid = make_grid(
        3,
        2,
        0.5F,
        {
            0, -1, 100,
            50, 51, 0
        });

    auto adapter = make_costmap(grid);

    EXPECT_TRUE(adapter.isReady());
    EXPECT_EQ(adapter.getSizeInCellsX(), 3U);
    EXPECT_EQ(adapter.getSizeInCellsY(), 2U);
    EXPECT_DOUBLE_EQ(adapter.getResolution(), 0.5);
    ASSERT_TRUE(adapter.gridMap().isReady());
    EXPECT_EQ(adapter.gridMap().width, 3U);
    EXPECT_EQ(adapter.gridMap().height, 2U);
    EXPECT_DOUBLE_EQ(adapter.gridMap().resolution, 0.5);
    EXPECT_EQ(adapter.gridMap().origin_x, 0.0);
    EXPECT_EQ(adapter.gridMap().origin_y, 0.0);
    EXPECT_EQ(adapter.gridMap().data, grid.data);

    EXPECT_EQ(adapter.getCost(0, 0), nav2_costmap_2d::FREE_SPACE);
    EXPECT_EQ(adapter.getCost(1, 0), nav2_costmap_2d::NO_INFORMATION);
    EXPECT_EQ(adapter.getCost(2, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
    EXPECT_LT(adapter.getCost(0, 1), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    EXPECT_EQ(adapter.getCost(1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);

    unsigned int mx = 0U;
    unsigned int my = 0U;
    EXPECT_TRUE(adapter.worldToMap(1.25, 0.25, mx, my));
    EXPECT_EQ(mx, 2U);
    EXPECT_EQ(my, 0U);

    double wx = 0.0;
    double wy = 0.0;
    adapter.mapToWorld(2U, 1U, wx, wy);
    EXPECT_DOUBLE_EQ(wx, 1.25);
    EXPECT_DOUBLE_EQ(wy, 0.75);

    EXPECT_TRUE(adapter.hasUnknownNeighbor(0U, 0U));
    const auto clearance = adapter.distanceToNearestObstacle(0U, 0U, 3);
    ASSERT_TRUE(clearance.has_value());
    EXPECT_NEAR(clearance.value(), std::sqrt(2.0) * 0.5, 1e-9);
}

TEST(CostmapAdapterTest, RejectsInvalidGridAndResetsReadiness)
{
    auto adapter = make_costmap(make_filled_grid(2, 2, 1.0F, 0));
    EXPECT_TRUE(adapter.isReady());

    auto invalid = make_filled_grid(2, 2, 1.0F, 0);
    invalid.data.pop_back();

    EXPECT_FALSE(adapter.updateFromOccupancyGrid(invalid));
    EXPECT_FALSE(adapter.isReady());
    EXPECT_FALSE(adapter.gridMap().isReady());
    EXPECT_TRUE(adapter.gridMap().data.empty());
    EXPECT_EQ(adapter.getSizeInCellsX(), 0U);
    EXPECT_EQ(adapter.getCost(0U, 0U), nav2_costmap_2d::NO_INFORMATION);
}

TEST(CostmapAdapterTest, PreservesOccupancyClassificationInGridMapSnapshot)
{
    const auto grid = make_grid(
        5,
        1,
        0.5F,
        {
            -1, 0, 1, 50, 51
        });

    auto adapter = make_costmap(grid);
    const auto & snapshot = adapter.gridMap();

    ASSERT_TRUE(snapshot.isReady());
    EXPECT_TRUE(snapshot.isUnknown(0U, 0U));
    EXPECT_TRUE(snapshot.isFree(1U, 0U));
    EXPECT_FALSE(snapshot.isObstacle(2U, 0U));
    EXPECT_FALSE(snapshot.isObstacle(3U, 0U));
    EXPECT_TRUE(snapshot.isObstacle(4U, 0U));
    EXPECT_EQ(adapter.isUnknown(0U, 0U), snapshot.isUnknown(0U, 0U));
    EXPECT_EQ(adapter.isFree(1U, 0U), snapshot.isFree(1U, 0U));
    EXPECT_EQ(adapter.isObstacle(4U, 0U), snapshot.isObstacle(4U, 0U));
}

TEST(FrontierDetectorTest, DetectsSafeFreeCellsWithUnknownNeighbors)
{
    auto grid = make_filled_grid(5, 5, 1.0F, 0);
    auto at = [](int row, int col) {return row * 5 + col;};
    grid.data[at(2, 2)] = -1;
    grid.data[at(1, 1)] = 100;

    auto adapter = make_costmap(grid);
    FrontierDetector detector(0);

    const auto frontiers = detector.detect_frontier_cells(adapter.gridMap());

    EXPECT_TRUE(contains_cell(frontiers, GridCell{1, 2}));
    EXPECT_TRUE(contains_cell(frontiers, GridCell{2, 1}));
    EXPECT_TRUE(contains_cell(frontiers, GridCell{2, 3}));
    EXPECT_TRUE(contains_cell(frontiers, GridCell{3, 2}));
    EXPECT_FALSE(contains_cell(frontiers, GridCell{1, 1}));
    EXPECT_FALSE(contains_cell(frontiers, GridCell{2, 2}));
}

TEST(FrontierDetectorTest, ClustersEightConnectedFrontiersAndComputesCentroids)
{
    FrontierDetector detector(0);
    auto adapter = make_costmap(make_filled_grid(8, 8, 1.0F, 0));

    const std::vector<GridCell> cells{
        {1, 1}, {1, 2}, {2, 2},
        {5, 5}, {6, 6}
    };

    auto clusters = detector.cluster_frontiers(adapter.gridMap(), cells);

    ASSERT_EQ(clusters.size(), 2U);
    std::sort(
        clusters.begin(),
        clusters.end(),
        [](const FrontierCluster & lhs, const FrontierCluster & rhs) {
            return lhs.cells.size() > rhs.cells.size();
        });

    EXPECT_EQ(clusters[0].cells.size(), 3U);
    EXPECT_EQ(clusters[0].centroid, (GridCell{1, 2}));
    EXPECT_EQ(clusters[1].cells.size(), 2U);
    EXPECT_EQ(clusters[1].centroid, (GridCell{6, 6}));
}

TEST(FrontierPrunerTest, AppliesMapConstraintsRetryStateAndInset)
{
    auto grid = make_filled_grid(10, 10, 1.0F, 0);
    auto adapter = make_costmap(grid);

    FrontierPruner pruner(
        1.0,
        2,
        3,
        1U,
        0,
        1,
        1.0);

    std::unordered_map<GridCell, int, GridCellHash> failed_goal_counts;
    failed_goal_counts[GridCell{5, 7}] = 1;
    std::unordered_set<GridCell, GridCellHash> blacklist{GridCell{1, 1}};
    FrontierPruningContext context;
    context.failed_goal_counts = &failed_goal_counts;
    context.goal_blacklist = &blacklist;
    FrontierPruningEnvironment environment;
    environment.frontier_map = &adapter.gridMap();

    const std::vector<FrontierCluster> clusters{
        make_cluster({{5, 7}, {5, 8}}, {5, 7}),
        make_cluster({{1, 1}}, {1, 1})
    };

    const auto candidates = pruner.prune_clusters(
        clusters,
        GridCell{5, 4},
        1.0,
        environment,
        context);

    ASSERT_FALSE(candidates.empty());
    const auto selected = std::find_if(
        candidates.begin(),
        candidates.end(),
        [](const FrontierCandidate & candidate) {
            return candidate.goal == GridCell{5, 6};
        });
    ASSERT_NE(selected, candidates.end());
    EXPECT_TRUE(selected->goal_inset_applied);
    EXPECT_EQ(selected->retry_count, 0);
    EXPECT_EQ(selected->cluster_centroid, (GridCell{5, 7}));

    EXPECT_TRUE(std::none_of(
        candidates.begin(),
        candidates.end(),
        [](const FrontierCandidate & candidate) {
            return candidate.goal == GridCell{1, 1};
        }));
}

TEST(FrontierScorerTest, NormalizesScoresAndAppliesPenalties)
{
    FrontierScoringWeights weights;
    weights.weight_distance = 1.0;
    weights.weight_cluster_size = 1.0;
    weights.weight_clearance = 1.0;
    weights.weight_retry_penalty = 1.0;
    weights.weight_unknown_risk_penalty = 1.0;
    weights.weight_information_gain = 0.0;
    weights.enable_clearance_score = true;
    weights.enable_unknown_risk_penalty = true;
    weights.enable_information_gain_score = false;
    weights.unknown_risk_threshold = 0.4;

    FrontierScorer scorer(weights, 2);

    const std::vector<FrontierCandidate> candidates{
        FrontierCandidate{
            GridCell{1, 1}, GridCell{1, 1}, 2U, 1.0,
            0, 1.0, 0.2, 0U, false, false, false, true, 0.0, {}},
        FrontierCandidate{
            GridCell{5, 5}, GridCell{5, 5}, 6U, 5.0,
            1, 3.0, 0.8, 1U, false, false, false, true, 0.0, {}}
    };

    const auto scored = scorer.score_candidates(candidates, std::nullopt);
    ASSERT_EQ(scored.size(), 2U);

    EXPECT_DOUBLE_EQ(scored[0].distance_score, 1.0);
    EXPECT_DOUBLE_EQ(scored[0].cluster_size_score, 0.0);
    EXPECT_NEAR(scored[0].clearance_score, 1.0 / 3.0, 1e-9);
    EXPECT_DOUBLE_EQ(scored[0].retry_penalty, 0.0);
    EXPECT_DOUBLE_EQ(scored[0].unknown_risk_penalty, 0.0);
    EXPECT_NEAR(scored[0].total_score, 1.0 + 1.0 / 3.0, 1e-9);

    EXPECT_DOUBLE_EQ(scored[1].distance_score, 0.0);
    EXPECT_DOUBLE_EQ(scored[1].cluster_size_score, 1.0);
    EXPECT_DOUBLE_EQ(scored[1].clearance_score, 1.0);
    EXPECT_DOUBLE_EQ(scored[1].retry_penalty, 0.5);
    EXPECT_NEAR(scored[1].unknown_risk_penalty, 2.0 / 3.0, 1e-9);
    EXPECT_NEAR(scored[1].total_score, 1.0 + 1.0 - 0.5 - 2.0 / 3.0, 1e-9);
}

TEST(FrontierSelectorTest, DefersSmallClustersAndHonorsReachability)
{
    auto adapter = make_costmap(make_filled_grid(20, 20, 1.0F, 0));

    FrontierScoringWeights weights;
    weights.weight_distance = 1.0;
    weights.weight_cluster_size = 0.0;
    weights.weight_information_gain = 0.0;
    weights.enable_information_gain_score = false;
    weights.enable_unknown_risk_penalty = false;

    FrontierSelector selector(
        1.0,
        2,
        weights,
        1U,
        3,
        0,
        0,
        1.0,
        FootprintCollisionCheckerConfig{},
        true,
        3U,
        true,
        rclcpp::get_logger("frontier_explorer_ros_test"));

    const std::vector<FrontierCluster> clusters{
        make_cluster({{10, 3}, {10, 4}}, {10, 3}),
        make_cluster({{10, 12}, {10, 13}, {10, 14}}, {10, 12})
    };

    const auto goal = selector.choose_best_frontier(
        clusters,
        GridCell{10, 0},
        1.0,
        adapter,
        nullptr,
        [](FrontierCandidate & candidate) {
            FrontierReachabilityResult result;
            result.checked = true;
            result.reachable = candidate.goal.col < 12;
            result.reason = result.reachable ? "" : "blocked";
            return result;
        });

    ASSERT_TRUE(goal.has_value());
    EXPECT_LT(goal->col, 12);
    EXPECT_EQ(goal->row, 10);
}

TEST(FrontierSelectorTest, BlacklistsFailedGoalsAndCanClearState)
{
    FrontierSelector selector(
        1.0,
        2,
        FrontierScoringWeights{},
        1U,
        3,
        0,
        0,
        1.0,
        FootprintCollisionCheckerConfig{},
        true,
        3U,
        false,
        rclcpp::get_logger("frontier_explorer_ros_test"));

    const GridCell goal{3, 4};
    selector.mark_goal_failed(goal);
    EXPECT_EQ(selector.retry_count_for_goal(goal), 1);
    EXPECT_FALSE(selector.is_goal_blacklisted(goal));

    selector.mark_goal_failed(goal);
    EXPECT_EQ(selector.retry_count_for_goal(goal), 2);
    EXPECT_TRUE(selector.is_goal_blacklisted(goal));
    EXPECT_EQ(selector.blacklisted_goals().size(), 1U);

    EXPECT_EQ(selector.clear_blacklist(), 1U);
    EXPECT_EQ(selector.retry_count_for_goal(goal), 0);
    EXPECT_FALSE(selector.is_goal_blacklisted(goal));
}

TEST(FrontierGoalProviderTest, RunsMapDetectorAndSelectorPath)
{
    auto grid = make_filled_grid(10, 10, 1.0F, 0);
    grid.data[5U * 10U + 5U] = -1;
    grid.header.frame_id = "map";

    FrontierGoalProvider provider(rclcpp::get_logger("frontier_goal_provider_test"));
    auto params = FrontierExplorerParams{};
    params.runtime.obstacle_search_radius_cells = 0;
    params.runtime.enable_reachability_filter = false;
    provider.configure(params);

    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>(grid);
    ASSERT_TRUE(provider.update_map(map, rclcpp::Time(1, 0)));

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 1.5;
    pose.pose.position.y = 1.5;
    provider.update_robot_pose(pose);

    const auto result = provider.compute_frontier_candidates(rclcpp::Time(2, 0));

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.state, ExplorationState::RUNNING);
    EXPECT_GT(result.raw_frontier_count, 0U);
    EXPECT_GT(result.candidate_count, 0U);
    EXPECT_FALSE(result.candidates.empty());
}

}  // namespace frontier_explorer
