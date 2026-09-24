#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "grid_map_ros/costmap_adapter.hpp"
#include "frontier_strategy_core/detector/frontier_detector.hpp"
#include "frontier_strategy_ros/adaptation/frontier_map_parameter_adapter.hpp"
#include "frontier_strategy_ros/frontier_goal_provider.hpp"
#include "frontier_strategy_ros/reachability/frontier_reachability_checker.hpp"
#include "frontier_strategy_core/scoring/frontier_scorer.hpp"
#include "frontier_strategy_core/selector/filters/frontier_pruner.hpp"
#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_strategy
{
using grid_map_ros::CostmapAdapter;

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
    CostmapAdapter adapter(rclcpp::get_logger("frontier_strategy_ros_test"));
    EXPECT_TRUE(adapter.updateFromOccupancyGrid(grid));
    return adapter;
}

std::shared_ptr<const robot_geometry_core::IRobotGeometryProvider>
make_robot_geometry_provider()
{
    return std::make_shared<robot_geometry_core::StaticRobotGeometryProvider>(
        robot_geometry_core::makeCircularCollisionEnvelope(
            0.1, 0.0, "base_link", "test", 1U));
}

class CountingRobotGeometryProvider final :
    public robot_geometry_core::IRobotGeometryProvider
{
public:
    robot_geometry_core::RobotCollisionEnvelope collisionEnvelope() const override
    {
        ++snapshot_calls;
        return robot_geometry_core::makeCircularCollisionEnvelope(
            0.1, 0.0, "base_link", "counting_test", 1U);
    }

    mutable std::size_t snapshot_calls{0U};
};

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

FrontierCandidatesResult compute_candidates(
    const nav_msgs::msg::OccupancyGrid & grid,
    const FrontierStrategyParams & params)
{
    FrontierGoalProvider provider(rclcpp::get_logger("frontier_goal_provider_parameter_test"));
    provider.configure(params, make_robot_geometry_provider());
    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>(grid);
    EXPECT_TRUE(provider.update_map(map, rclcpp::Time(1, 0)));

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 1.5;
    pose.pose.position.y = 1.5;
    provider.update_robot_pose(pose);
    return provider.compute_frontier_candidates(rclcpp::Time(2, 0));
}

}  // 命名空间

TEST(FrontierMapParameterAdapterTest, ConvertsMetricParametersUsingMapResolution)
{
    FrontierStrategyParams params;
    params.runtime.obstacle_search_radius_cells = 9;
    params.runtime.min_frontier_cluster_size = 9;
    params.selection.small_cluster_size_threshold = 9U;
    params.pruner.candidate_unknown_margin_cells = 9;
    params.pruner.candidate_goal_inset_cells = 9;
    params.pruner.cleanup_goal_inset_cells = 9;
    params.map_adaptation.enabled = true;
    params.map_adaptation.obstacle_clearance_m = 0.05;
    params.map_adaptation.min_frontier_length_m = 0.10;
    params.map_adaptation.small_frontier_length_m = 0.25;
    params.map_adaptation.candidate_unknown_margin_m = 0.10;
    params.map_adaptation.candidate_goal_inset_m = 0.15;
    params.map_adaptation.cleanup_goal_inset_m = 0.0;

    const auto adapted = adapt_strategy_params_to_map_resolution(params, 0.05);

    EXPECT_EQ(adapted.runtime.obstacle_search_radius_cells, 1);
    EXPECT_EQ(adapted.runtime.min_frontier_cluster_size, 2);
    EXPECT_EQ(adapted.selection.small_cluster_size_threshold, 5U);
    EXPECT_EQ(adapted.pruner.candidate_unknown_margin_cells, 2);
    EXPECT_EQ(adapted.pruner.candidate_goal_inset_cells, 3);
    EXPECT_EQ(adapted.pruner.cleanup_goal_inset_cells, 0);
    EXPECT_EQ(adapted.pruner.min_cluster_size, 2U);

    const auto coarser_map = adapt_strategy_params_to_map_resolution(params, 0.10);
    EXPECT_EQ(coarser_map.runtime.obstacle_search_radius_cells, 1);
    EXPECT_EQ(coarser_map.runtime.min_frontier_cluster_size, 1);
    EXPECT_EQ(coarser_map.selection.small_cluster_size_threshold, 3U);
    EXPECT_EQ(coarser_map.pruner.candidate_unknown_margin_cells, 1);
    EXPECT_EQ(coarser_map.pruner.candidate_goal_inset_cells, 2);
}

TEST(FrontierMapParameterAdapterTest, KeepsCellParametersWhenAdaptationIsDisabled)
{
    FrontierStrategyParams params;
    params.map_adaptation.enabled = false;
    params.runtime.obstacle_search_radius_cells = 4;
    params.runtime.min_frontier_cluster_size = 6;
    params.pruner.min_cluster_size = 6U;
    params.pruner.candidate_goal_inset_cells = 7;

    const auto adapted = adapt_strategy_params_to_map_resolution(params, 0.05);

    EXPECT_EQ(adapted.runtime.obstacle_search_radius_cells, 4);
    EXPECT_EQ(adapted.runtime.min_frontier_cluster_size, 6);
    EXPECT_EQ(adapted.pruner.min_cluster_size, 6U);
    EXPECT_EQ(adapted.pruner.candidate_goal_inset_cells, 7);
}

TEST(FrontierMapParameterAdapterTest, KeepsCellParametersForInvalidResolution)
{
    FrontierStrategyParams params;
    params.map_adaptation.enabled = true;
    params.runtime.obstacle_search_radius_cells = 3;
    params.pruner.candidate_unknown_margin_cells = 4;

    const auto adapted = adapt_strategy_params_to_map_resolution(params, 0.0);

    EXPECT_EQ(adapted.runtime.obstacle_search_radius_cells, 3);
    EXPECT_EQ(adapted.pruner.candidate_unknown_margin_cells, 4);
}

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
    EXPECT_LT(adapter.getCost(1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);

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
    EXPECT_NEAR(clearance.value(), 2.0 * 0.5, 1e-9);
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

TEST(CostmapAdapterTest, PreservesOccupancyClassificationInGridMap)
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
    EXPECT_FALSE(snapshot.isObstacle(4U, 0U));
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

TEST(FrontierGoalProviderTest, RunsMapDetectorAndSelectorPath)
{
    auto grid = make_filled_grid(10, 10, 1.0F, 0);
    grid.data[5U * 10U + 5U] = -1;
    grid.header.frame_id = "map";

    FrontierGoalProvider provider(rclcpp::get_logger("frontier_goal_provider_test"));
    auto params = FrontierStrategyParams{};
    params.map_adaptation.enabled = false;
    params.runtime.obstacle_search_radius_cells = 0;
    params.runtime.enable_reachability_filter = false;
    const auto geometry_provider = std::make_shared<CountingRobotGeometryProvider>();
    provider.configure(params, geometry_provider);

    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>(grid);
    ASSERT_TRUE(provider.update_map(map, rclcpp::Time(1, 0)));

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 1.5;
    pose.pose.position.y = 1.5;
    provider.update_robot_pose(pose);

    const auto result = provider.compute_frontier_candidates(rclcpp::Time(2, 0));

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.state, ExplorationStatus::RUNNING);
    EXPECT_GT(result.raw_frontier_count, 0U);
    EXPECT_GT(result.candidate_count, 0U);
    EXPECT_FALSE(result.candidates.empty());
    EXPECT_GT(geometry_provider->snapshot_calls, 0U);

    bool saw_positive_gain = false;
    for (const auto & message_candidate : result.candidates) {
        const auto core_candidate = std::find_if(
            result.visualization.scored_candidates.begin(),
            result.visualization.scored_candidates.end(),
            [&message_candidate](const ScoredFrontierCandidate & candidate) {
                return candidate.candidate.goal.row == message_candidate.goal_row &&
                       candidate.candidate.goal.col == message_candidate.goal_col;
            });
        ASSERT_NE(core_candidate, result.visualization.scored_candidates.end());
        EXPECT_EQ(
            message_candidate.information_gain_valid,
            core_candidate->candidate.information_gain_valid);
        EXPECT_NEAR(
            message_candidate.information_gain,
            core_candidate->candidate.information_gain,
            1e-6);
        saw_positive_gain |= message_candidate.information_gain_valid &&
            message_candidate.information_gain > 0.0F;
    }
    EXPECT_TRUE(saw_positive_gain);
}

TEST(FrontierGoalProviderTest, AppliesAllInformationGainParametersToProductionPath)
{
    auto grid = make_filled_grid(10, 10, 1.0F, 0);
    grid.data[5U * 10U + 5U] = -1;
    grid.header.frame_id = "map";

    auto base = FrontierStrategyParams{};
    base.map_adaptation.enabled = false;
    base.runtime.obstacle_search_radius_cells = 0;
    base.runtime.enable_reachability_filter = false;
    base.pruner.minimum_information_gain_m2 = 0.0;

    auto disabled = base;
    disabled.pruner.enable_information_gain = false;
    const auto disabled_result = compute_candidates(grid, disabled);
    ASSERT_FALSE(disabled_result.candidates.empty());
    EXPECT_TRUE(std::all_of(
        disabled_result.candidates.begin(),
        disabled_result.candidates.end(),
        [](const auto & candidate) {
            return !candidate.information_gain_valid && candidate.information_gain == 0.0F;
        }));

    auto short_range = base;
    short_range.pruner.enable_information_gain = true;
    short_range.pruner.information_gain_sensor_range_m = 0.5;
    const auto short_range_result = compute_candidates(grid, short_range);
    ASSERT_FALSE(short_range_result.candidates.empty());
    EXPECT_TRUE(std::all_of(
        short_range_result.candidates.begin(),
        short_range_result.candidates.end(),
        [](const auto & candidate) {
            return candidate.information_gain_valid && candidate.information_gain == 0.0F;
        }));

    auto zero_weight = base;
    zero_weight.pruner.information_gain_sensor_range_m = 3.0;
    zero_weight.scorer.weights.weight_information_gain = 0.0;
    const auto zero_weight_result = compute_candidates(grid, zero_weight);
    ASSERT_FALSE(zero_weight_result.candidates.empty());

    auto weighted = zero_weight;
    weighted.scorer.weights.weight_information_gain = 1.0;
    weighted.scorer.weights.information_gain_saturation_area_m2 = 1.0;
    const auto weighted_result = compute_candidates(grid, weighted);
    ASSERT_FALSE(weighted_result.candidates.empty());

    const auto weighted_candidate = std::find_if(
        weighted_result.candidates.begin(),
        weighted_result.candidates.end(),
        [](const auto & candidate) {return candidate.information_gain > 0.0F;});
    ASSERT_NE(weighted_candidate, weighted_result.candidates.end());
    const auto zero_weight_candidate = std::find_if(
        zero_weight_result.candidates.begin(),
        zero_weight_result.candidates.end(),
        [&weighted_candidate](const auto & candidate) {
            return candidate.goal_row == weighted_candidate->goal_row &&
                   candidate.goal_col == weighted_candidate->goal_col;
        });
    ASSERT_NE(zero_weight_candidate, zero_weight_result.candidates.end());
    EXPECT_GT(weighted_candidate->score, zero_weight_candidate->score);

    auto faster_saturation = weighted;
    faster_saturation.scorer.weights.information_gain_saturation_area_m2 = 0.1;
    const auto faster_saturation_result = compute_candidates(grid, faster_saturation);
    const auto faster_candidate = std::find_if(
        faster_saturation_result.candidates.begin(),
        faster_saturation_result.candidates.end(),
        [&weighted_candidate](const auto & candidate) {
            return candidate.goal_row == weighted_candidate->goal_row &&
                   candidate.goal_col == weighted_candidate->goal_col;
        });
    ASSERT_NE(faster_candidate, faster_saturation_result.candidates.end());
    EXPECT_GT(faster_candidate->score, weighted_candidate->score);

    auto hard_minimum = weighted;
    hard_minimum.pruner.minimum_information_gain_m2 = 1000.0;
    const auto hard_minimum_result = compute_candidates(grid, hard_minimum);
    EXPECT_TRUE(hard_minimum_result.candidates.empty());
}

TEST(FrontierGoalProviderTest, RequiresStableNoFrontierCyclesBeforeCompletion)
{
    auto grid = make_filled_grid(6, 6, 1.0F, 0);
    grid.header.frame_id = "map";

    FrontierGoalProvider provider(rclcpp::get_logger("frontier_goal_provider_stable_test"));
    auto params = FrontierStrategyParams{};
    params.map_adaptation.enabled = false;
    params.runtime.obstacle_search_radius_cells = 0;
    params.runtime.enable_reachability_filter = false;
    params.runtime.stable_no_frontier_cycles = 3;
    provider.configure(params, make_robot_geometry_provider());

    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>(grid);
    ASSERT_TRUE(provider.update_map(map, rclcpp::Time(1, 0)));

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 2.5;
    pose.pose.position.y = 2.5;
    provider.update_robot_pose(pose);

    const auto first = provider.compute_frontier_candidates(rclcpp::Time(2, 0));
    EXPECT_FALSE(first.exploration_complete);
    EXPECT_TRUE(first.recoverable);
    EXPECT_EQ(first.reason_text, "WAITING_FOR_STABLE_NO_FRONTIER");
    EXPECT_EQ(first.stable_no_frontier_cycles, 1);

    const auto second = provider.compute_frontier_candidates(rclcpp::Time(2, 1));
    EXPECT_FALSE(second.exploration_complete);
    EXPECT_TRUE(second.recoverable);
    EXPECT_EQ(second.stable_no_frontier_cycles, 2);

    const auto third = provider.compute_frontier_candidates(rclcpp::Time(2, 2));
    EXPECT_TRUE(third.exploration_complete);
    EXPECT_FALSE(third.recoverable);
    EXPECT_EQ(third.reason_text, "NO_FRONTIER_FOUND");
    EXPECT_EQ(third.stable_no_frontier_cycles, 3);
}

}  // 命名空间 frontier_strategy
