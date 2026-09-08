#include "frontier_explorer_core/selector/filters/frontier_pruner.hpp"
#include "frontier_explorer_core/selector/frontier_selection_policy.hpp"
#include "frontier_explorer_core/types/grid_map.hpp"

int main()
{
  frontier_explorer::GridMap map;
  frontier_explorer::FrontierPruner pruner(0.5, 2, 3, 1U, 2, 2, 0.4);
  frontier_explorer::FrontierSelectionPolicy policy;
  frontier_explorer::FrontierPruningEnvironment environment;
  environment.frontier_map = &map;

  (void)pruner;
  (void)policy;
  return environment.frontier_map->isReady() ? 1 : 0;
}
