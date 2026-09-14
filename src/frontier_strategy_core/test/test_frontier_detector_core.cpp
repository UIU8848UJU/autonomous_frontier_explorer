#include <algorithm>
#include <cstdint>
#include <vector>

#include "frontier_strategy_core/detector/frontier_detector.hpp"
#include "grid_map_core/types/grid_map.hpp"
#include "gtest/gtest.h"

namespace frontier_strategy
{
namespace
{

grid_map_core::GridMap make_map(unsigned int width, unsigned int height, std::int8_t value)
{
    grid_map_core::GridMap map;
    map.width = width;
    map.height = height;
    map.resolution = 1.0;
    map.data.assign(static_cast<std::size_t>(width) * height, value);
    return map;
}

}  // 命名空间

TEST(FrontierDetectorCore, RejectsInvalidMapWithoutROS)
{
    const grid_map_core::GridMap map;
    const FrontierDetector detector(0);

    EXPECT_FALSE(map.isReady());
    EXPECT_TRUE(detector.detect_frontier_cells(map).empty());
}

TEST(FrontierDetectorCore, DetectsAndClustersFrontiersFromDomainMap)
{
    auto map = make_map(5U, 5U, 0);
    map.data[2U * map.width + 2U] = -1;

    const FrontierDetector detector(0);
    const auto cells = detector.detect_frontier_cells(map);
    const auto clusters = detector.cluster_frontiers(map, cells);

    EXPECT_EQ(cells.size(), 8U);
    ASSERT_EQ(clusters.size(), 1U);
    EXPECT_EQ(clusters.front().cells.size(), 8U);
    EXPECT_EQ(clusters.front().centroid, (GridCell{2, 2}));
}

TEST(FrontierDetectorCore, DetectsFrontierOnMapBoundary)
{
    auto map = make_map(3U, 3U, 0);
    map.data[0U] = -1;

    const FrontierDetector detector(1);
    const auto cells = detector.detect_frontier_cells(map);

    EXPECT_TRUE(std::find(cells.begin(), cells.end(), GridCell{0, 1}) != cells.end());
}

}  // 命名空间 frontier_strategy
