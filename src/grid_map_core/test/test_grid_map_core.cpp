#include <cstdint>

#include <gtest/gtest.h>

#include "grid_map_core/types/grid_map.hpp"

namespace
{

grid_map_core::GridMap make_map()
{
    grid_map_core::GridMap map;
    map.width = 4U;
    map.height = 3U;
    map.resolution = 0.5;
    map.origin_x = -1.0;
    map.origin_y = 2.0;
    map.data.assign(map.width * map.height, static_cast<std::int8_t>(0));
    map.data[1U] = 100;
    map.data[2U] = 60;
    map.data[8U] = -1;
    return map;
}

}  // namespace

TEST(GridMapCoreTest, ValidatesMapShape)
{
    auto map = make_map();
    EXPECT_TRUE(map.isReady());

    map.data.pop_back();
    EXPECT_FALSE(map.isReady());
}

TEST(GridMapCoreTest, ConvertsBetweenGridAndWorldCoordinates)
{
    const auto map = make_map();
    double world_x = 0.0;
    double world_y = 0.0;
    ASSERT_TRUE(map.mapToWorld(grid_map_core::GridCell{1, 2}, world_x, world_y));
    EXPECT_DOUBLE_EQ(world_x, -1.0 + 2.5 * 0.5);
    EXPECT_DOUBLE_EQ(world_y, 2.0 + 1.5 * 0.5);

    grid_map_core::GridCell cell;
    ASSERT_TRUE(map.worldToMap(world_x, world_y, cell));
    EXPECT_EQ(cell, (grid_map_core::GridCell{1, 2}));
}

TEST(GridMapCoreTest, ClassifiesCellsAndFindsObstacleDistance)
{
    const auto map = make_map();
    EXPECT_TRUE(map.isFree(0U, 0U));
    EXPECT_TRUE(map.isObstacle(1U, 0U));
    EXPECT_FALSE(map.isObstacle(2U, 0U));
    EXPECT_TRUE(map.isUnknown(0U, 2U));

    const auto distance = map.distanceToNearestObstacle(grid_map_core::GridCell{0, 0}, 2);
    ASSERT_TRUE(distance.has_value());
    EXPECT_DOUBLE_EQ(distance.value(), 0.5);
}
