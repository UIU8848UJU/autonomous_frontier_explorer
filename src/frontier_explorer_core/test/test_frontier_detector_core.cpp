#include <cstdint>
#include <vector>

#include "frontier_explorer_core/detector/frontier_detector.hpp"
#include "frontier_explorer_core/types/grid_map.hpp"
#include "gtest/gtest.h"

namespace frontier_explorer
{
namespace
{

GridMap make_map(unsigned int width, unsigned int height, std::int8_t value)
{
    GridMap map;
    map.width = width;
    map.height = height;
    map.resolution = 1.0;
    map.data.assign(static_cast<std::size_t>(width) * height, value);
    return map;
}

}  // namespace

TEST(FrontierDetectorCore, RejectsInvalidMapWithoutROS)
{
    const GridMap map;
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

}  // namespace frontier_explorer
