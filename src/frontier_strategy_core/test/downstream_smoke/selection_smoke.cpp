#include "frontier_strategy_core/selector/filters/frontier_pruner.hpp"
#include "frontier_strategy_core/selector/frontier_selection_policy.hpp"
#include "frontier_strategy_core/types/grid_map.hpp"

int main()
{
  grid_map_core::GridMap map;
  frontier_strategy::FrontierPruner pruner(0.5, 2, 3, 1U, 2, 2, 0.4);
  frontier_strategy::FrontierSelectionPolicy policy;
  frontier_strategy::FrontierPruningEnvironment environment;
  environment.frontier_map = &map;

  (void)pruner;
  (void)policy;
  return environment.frontier_map->isReady() ? 1 : 0;
}
