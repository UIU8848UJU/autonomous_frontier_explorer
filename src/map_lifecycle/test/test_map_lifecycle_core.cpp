#include <cstdint>
#include <initializer_list>

#include "gtest/gtest.h"
#include "map_lifecycle_core/map_lifecycle_core.hpp"

namespace map_lifecycle
{
namespace
{
grid_map_core::GridMap make_map(
  std::uint32_t width,
  std::uint32_t height,
  double resolution,
  std::initializer_list<std::int8_t> data)
{
  grid_map_core::GridMap map;
  map.width = width;
  map.height = height;
  map.resolution = resolution;
  map.origin_x = 1.25;
  map.origin_y = -0.5;
  map.data = data;
  return map;
}

ExplorationCompletedEvent make_completed_event()
{
  ExplorationCompletedEvent event;
  event.detail = "test";
  return event;
}
}  // 匿名命名空间

TEST(MapLifecycleCoreTest, UpdatesMapStatisticsAndKeepsLatestMap)
{
  MapLifecycleCore core;
  const auto map = make_map(3U, 2U, 0.05, {0, -1, 100, -1, 0, 0});

  core.update_map(map);

  EXPECT_EQ(core.latest_map().width, 3U);
  EXPECT_EQ(core.latest_map().height, 2U);
  EXPECT_DOUBLE_EQ(core.latest_map().origin_x, 1.25);
  EXPECT_DOUBLE_EQ(core.latest_map().origin_y, -0.5);
  EXPECT_EQ(core.latest_map().data, map.data);
  EXPECT_DOUBLE_EQ(core.map_statistics().resolution, 0.05);
  EXPECT_DOUBLE_EQ(core.map_statistics().unknown_ratio, 2.0 / 6.0);
  EXPECT_DOUBLE_EQ(core.map_statistics().explored_ratio, 4.0 / 6.0);
  EXPECT_DOUBLE_EQ(core.map_statistics().free_ratio, 3.0 / 6.0);
  EXPECT_DOUBLE_EQ(core.map_statistics().occupied_ratio, 1.0 / 6.0);
  EXPECT_TRUE(core.map_statistics().valid);
  EXPECT_EQ(core.state(), MapLifecycleState::ACTIVE);
}

TEST(MapLifecycleCoreTest, EmptyMapHasUnknownRatioOneAndIsInvalid)
{
  MapLifecycleCore core;
  core.update_map(make_map(0U, 0U, 0.05, {}));

  EXPECT_DOUBLE_EQ(core.map_statistics().unknown_ratio, 1.0);
  EXPECT_DOUBLE_EQ(core.map_statistics().explored_ratio, 0.0);
  EXPECT_FALSE(core.map_statistics().valid);
  EXPECT_EQ(core.state(), MapLifecycleState::EMPTY);
  EXPECT_FALSE(core.should_save(true));
}

TEST(MapLifecycleCoreTest, NonPositiveResolutionMakesMapInvalid)
{
  MapLifecycleCore core;
  core.update_map(make_map(1U, 1U, 0.0, {0}));

  EXPECT_FALSE(core.map_statistics().valid);
  EXPECT_EQ(core.state(), MapLifecycleState::EMPTY);
}

TEST(MapLifecycleCoreTest, RejectsMismatchedMapDataLength)
{
  MapLifecycleCore core;
  core.update_map(make_map(2U, 2U, 0.05, {0}));

  EXPECT_FALSE(core.map_statistics().valid);
  core.handle_exploration_completed(make_completed_event());
  EXPECT_FALSE(core.should_save(true));

  core.update_map(make_map(1U, 1U, 0.05, {0, 0}));
  EXPECT_FALSE(core.map_statistics().valid);
  EXPECT_FALSE(core.should_save(true));
}

TEST(MapLifecycleCoreTest, SaveRequiresValidMapAndCompletedExploration)
{
  MapLifecycleCore core;
  core.update_map(make_map(1U, 1U, 0.05, {0}));

  EXPECT_FALSE(core.should_save(true));
  core.handle_exploration_completed(make_completed_event());
  EXPECT_EQ(core.state(), MapLifecycleState::READY);
  EXPECT_TRUE(core.should_save(true));
  EXPECT_FALSE(core.should_save(false));
}

TEST(MapLifecycleCoreTest, FailedSaveCanBeRetriedAndSuccessfulSaveStopsRetries)
{
  MapLifecycleCore core;
  core.update_map(make_map(1U, 1U, 0.05, {0}));
  core.handle_exploration_completed(make_completed_event());

  EXPECT_TRUE(core.completion_detected());

  core.mark_save_requested();
  EXPECT_EQ(core.state(), MapLifecycleState::SAVING);
  EXPECT_FALSE(core.should_save(true));
  core.record_save_result(false, "");
  EXPECT_EQ(core.state(), MapLifecycleState::FAILED);
  EXPECT_TRUE(core.completion_detected());
  EXPECT_TRUE(core.should_save(true));

  core.mark_save_requested();
  core.record_save_result(true, "/tmp/map");
  EXPECT_EQ(core.state(), MapLifecycleState::SAVED);
  EXPECT_EQ(core.saved_map_url(), "/tmp/map");
  EXPECT_FALSE(core.should_save(true));
}

}  // 命名空间 map_lifecycle
