#include <cstdint>
#include <initializer_list>

#include "gtest/gtest.h"
#include "map_manager_core/map_manager_core.hpp"

namespace map_manager
{
namespace
{
MapSnapshot make_map(
  std::uint32_t width,
  std::uint32_t height,
  double resolution,
  std::initializer_list<std::int8_t> data)
{
  MapSnapshot map;
  map.width = width;
  map.height = height;
  map.resolution = resolution;
  map.data = data;
  return map;
}

ExplorationState make_state(ExplorationPhase phase)
{
  ExplorationState message;
  message.phase = phase;
  message.detail = "test";
  return message;
}
}  // namespace

TEST(MapManagerCoreTest, UpdatesMapStatisticsAndKeepsLatestMap)
{
  MapManagerCore core;
  const auto map = make_map(3U, 2U, 0.05, {0, -1, 100, -1, 0, 0});

  core.update_map(map);

  EXPECT_EQ(core.latest_map().width, 3U);
  EXPECT_EQ(core.latest_map().height, 2U);
  EXPECT_EQ(core.latest_map().data, map.data);
  EXPECT_DOUBLE_EQ(core.map_statistics().resolution, 0.05);
  EXPECT_DOUBLE_EQ(core.map_statistics().unknown_ratio, 2.0 / 6.0);
  EXPECT_TRUE(core.map_statistics().valid);
}

TEST(MapManagerCoreTest, EmptyMapHasUnknownRatioOneAndIsInvalid)
{
  MapManagerCore core;
  core.update_map(make_map(0U, 0U, 0.05, {}));

  EXPECT_DOUBLE_EQ(core.map_statistics().unknown_ratio, 1.0);
  EXPECT_FALSE(core.map_statistics().valid);
  EXPECT_FALSE(core.should_save(true));
}

TEST(MapManagerCoreTest, RejectsMismatchedMapDataLength)
{
  MapManagerCore core;
  core.update_map(make_map(2U, 2U, 0.05, {0}));

  EXPECT_FALSE(core.map_statistics().valid);
  core.update_exploration_state(make_state(kExplorationCompleted));
  EXPECT_FALSE(core.should_save(true));

  core.update_map(make_map(1U, 1U, 0.05, {0, 0}));
  EXPECT_FALSE(core.map_statistics().valid);
  EXPECT_FALSE(core.should_save(true));
}

TEST(MapManagerCoreTest, SaveRequiresValidMapAndCompletedExploration)
{
  MapManagerCore core;
  core.update_map(make_map(1U, 1U, 0.05, {0}));

  EXPECT_FALSE(core.should_save(true));
  core.update_exploration_state(make_state(kExplorationCompleted));
  EXPECT_TRUE(core.should_save(true));
  EXPECT_FALSE(core.should_save(false));
}

TEST(MapManagerCoreTest, FailedSaveCanBeRetriedAndSuccessfulSaveStopsRetries)
{
  MapManagerCore core;
  core.update_map(make_map(1U, 1U, 0.05, {0}));
  core.update_exploration_state(make_state(kExplorationCompleted));

  EXPECT_FALSE(core.completion_detected());
  core.mark_completion_detected();
  EXPECT_TRUE(core.completion_detected());

  core.mark_save_requested();
  EXPECT_FALSE(core.should_save(true));
  core.record_save_result(false, "");
  EXPECT_TRUE(core.completion_detected());
  EXPECT_TRUE(core.should_save(true));

  core.mark_save_requested();
  core.record_save_result(true, "/tmp/map");
  EXPECT_TRUE(core.save_requested());
  EXPECT_TRUE(core.save_succeeded());
  EXPECT_EQ(core.saved_map_url(), "/tmp/map");
  EXPECT_FALSE(core.should_save(true));
}

}  // namespace map_manager
