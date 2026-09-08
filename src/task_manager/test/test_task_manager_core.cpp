#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "task_flow/task_flow.hpp"

namespace task_manager
{
namespace
{
ExplorationEvent make_event(ExplorationState state, std::string detail = {})
{
  ExplorationEvent event;
  event.state = state;
  event.detail = std::move(detail);
  return event;
}
}  // namespace

TEST(TaskFlowTest, StartsAndRejectsDuplicateMappingFlow)
{
  TaskFlow flow;

  EXPECT_TRUE(flow.start_mapping_flow());
  EXPECT_EQ(flow.state(), TaskManagerState::EXPLORING);
  EXPECT_TRUE(flow.context().exploration_running);
  EXPECT_FALSE(flow.context().map_ready);
  const auto state_before_duplicate = flow.state();
  const auto context_before_duplicate = flow.context();
  EXPECT_FALSE(flow.start_mapping_flow());
  EXPECT_EQ(flow.state(), state_before_duplicate);
  EXPECT_EQ(flow.context().map_ready, context_before_duplicate.map_ready);
  EXPECT_EQ(
    flow.context().exploration_running,
    context_before_duplicate.exploration_running);
  EXPECT_EQ(flow.context().last_error, context_before_duplicate.last_error);
}

TEST(TaskFlowTest, UpdatesRunningAndCompletedExplorationStates)
{
  TaskFlow flow;

  flow.update_exploration_state(make_event(ExplorationState::RUNNING, "frontiers"));
  EXPECT_EQ(flow.state(), TaskManagerState::EXPLORING);
  EXPECT_TRUE(flow.context().exploration_running);
  EXPECT_EQ(flow.context().last_exploration_state, "RUNNING - frontiers");

  flow.update_exploration_state(make_event(ExplorationState::COMPLETED));
  EXPECT_EQ(flow.state(), TaskManagerState::MAPPING_DONE);
  EXPECT_FALSE(flow.context().exploration_running);
  EXPECT_TRUE(flow.context().map_ready);
  EXPECT_EQ(flow.context().last_exploration_state, "COMPLETED");
}

TEST(TaskFlowTest, StoppedAndIdleExplorationReturnToIdle)
{
  TaskFlow flow;
  flow.start_mapping_flow();

  flow.update_exploration_state(make_event(ExplorationState::STOPPED));
  EXPECT_EQ(flow.state(), TaskManagerState::IDLE);
  EXPECT_FALSE(flow.context().exploration_running);

  flow.start_mapping_flow();
  flow.update_exploration_state(make_event(ExplorationState::IDLE));
  EXPECT_EQ(flow.state(), TaskManagerState::IDLE);
  EXPECT_FALSE(flow.context().exploration_running);
}

TEST(TaskFlowTest, StuckExplorationFailsWithProvidedOrDefaultError)
{
  TaskFlow flow;

  flow.update_exploration_state(make_event(ExplorationState::STUCK, "no safe frontier"));
  EXPECT_EQ(flow.state(), TaskManagerState::FAILED);
  EXPECT_FALSE(flow.context().exploration_running);
  EXPECT_EQ(flow.context().last_error, "no safe frontier");

  flow.update_exploration_state(make_event(ExplorationState::STUCK));
  EXPECT_EQ(flow.context().last_error, "Exploration reported STUCK state.");

  EXPECT_TRUE(flow.start_mapping_flow());
  EXPECT_EQ(flow.state(), TaskManagerState::EXPLORING);
  EXPECT_TRUE(flow.context().exploration_running);
  EXPECT_FALSE(flow.context().map_ready);
  EXPECT_TRUE(flow.context().last_error.empty());
}

TEST(TaskFlowTest, StopAllReportsWhetherAnActiveFlowExisted)
{
  TaskFlow flow;

  EXPECT_FALSE(flow.stop_all());
  flow.start_mapping_flow();
  EXPECT_TRUE(flow.stop_all());
  EXPECT_EQ(flow.state(), TaskManagerState::IDLE);
  EXPECT_FALSE(flow.context().exploration_running);
  EXPECT_FALSE(flow.stop_all());
}

TEST(TaskFlowTest, MapSavedMarksMappingDoneAndClearsError)
{
  TaskFlow flow;
  flow.start_mapping_flow();
  flow.set_error("temporary failure");

  EXPECT_TRUE(flow.mark_map_saved());
  EXPECT_EQ(flow.state(), TaskManagerState::MAPPING_DONE);
  EXPECT_TRUE(flow.context().map_ready);
  EXPECT_FALSE(flow.context().exploration_running);
  EXPECT_TRUE(flow.context().last_error.empty());
}

}  // namespace task_manager
